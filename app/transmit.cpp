extern "C" {
#include "ethercat.h"
#include "motor_control.h"
#include "transmit.h"
}

//下面这都是C++的头文件，不能在transmit.h中进行#include，否则会导致编译错误
#include "queue.h"
#include <sys/time.h>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <thread>
#include <mutex>

#define EC_TIMEOUT_M
std::mutex motor_data_mutex; //全局互斥锁，用于保护motorDate_recv数组
std::mutex message_mutex; //全局互斥锁，用于保护Tx_Message数组
spsc_queue<EtherCAT_Msg_ptr, capacity<10> > messages[SLAVE_NUMBER];
std::atomic<bool> running{false};
std::thread runThread; //Ethercat任务线程
YKSMotorData motorDate_recv[TOTAL_MOTOR_NUMBER]; //一个存放所有YKS电机的状态信息以及期望状态信息的数组
YKS_IMUData imuData_recv;
char IO_map[4096];
OSAL_THREAD_HANDLE checkThread;
int expectedWKC;
boolean need_lf;
volatile int wkc;
boolean inOP;
uint8 current_group = 0;
uint64_t num;
bool isConfig[SLAVE_NUMBER]{false};

#define EC_TIMEOUT_MON 500

void EtherCAT_Data_Get();

//用于对EtherCAT总线进行初始化，并自动配置从站，从站会按照串联连接的顺序进行编号
int CAT_Init(const char *device_name) {
    printf("\033[32mEthercat SOEM 主站启动\033[0m\n");
    const char *ifconfig_name = device_name;
    EtherCAT_Init(ifconfig_name);
    if (ec_slavecount <= 0) {
        printf("\033[31m未找到从站, 程序退出！\033[0m");
        // exit(1);
        return false;
    }
    printf("\033[36m从站数量： %d\033[0m\r\n", ec_slavecount);
    startRun();
    return true;
}

static void degraded_handler() {
    printf("[EtherCAT Error] Logging error...\n");
    const time_t current_time = time(nullptr);
    char *time_str = ctime(&current_time);
    printf("E_STOP. EtherCAT became degraded at %s.\n", time_str);
    printf("[EtherCAT Error] Stopping RT process.\n");
}

static int run_ethercat(const char *if_name) {
    need_lf = FALSE;
    inOP = FALSE;

    num = 1;

    /* initialise SOEM, bind socket to if_name */
    if (ec_init(if_name)) {
        printf("\033[32m[EtherCAT Init] Initialization on device %s succeeded.\n\033[0m", if_name);
        /* find and auto-config slaves */

        if (ec_config_init(FALSE) > 0) {
            printf("[EtherCAT Init] %d slaves found and configured.\n", ec_slavecount);
            if (ec_slavecount < SLAVE_NUMBER) {
                printf("[RT EtherCAT] Warning: Expected %d slaves, found %d.\n", SLAVE_NUMBER, ec_slavecount);
            }

            for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
                ec_slave[slave_idx + 1].CoEdetails &= ~ECT_COEDET_SDOCA;

            ec_config_map(&IO_map);
            ec_configdc();

            printf("[EtherCAT Init] Mapped slaves.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * SLAVE_NUMBER);

            for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++) {
                printf("[SLAVE %d]\n", slave_idx);
                printf("  IN  %d bytes, %d bits\n", ec_slave[slave_idx].Ibytes, ec_slave[slave_idx].Ibits);
                printf("  OUT %d bytes, %d bits\n", ec_slave[slave_idx].Obytes, ec_slave[slave_idx].Obits);
                printf("\n");
            }

            int oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0))
                oloop = 1;
            if (oloop > 8)
                oloop = 8;
            int iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0))
                iloop = 1;
            if (iloop > 8)
                iloop = 8;

            printf("[EtherCAT Init] segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0],
                   ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

            printf("[EtherCAT Init] Requesting operational state for all slaves...\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("[EtherCAT Init] Calculated work_counter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            int chk = 40;
            /* wait for all slaves to reach OP state */
            do {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
                printf("[EtherCAT Init] Operational state reached for all slaves.\n");
                inOP = TRUE;
                return 1;
            } else {
                printf("[EtherCAT Error] Not all slaves reached operational state.\n");
                ec_readstate();
                for (int i = 1; i <= ec_slavecount; i++) {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                        printf("[EtherCAT Error] Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                               ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
        } else {
            printf("[EtherCAT Error] No slaves found!\n");
        }
    } else {
        printf("[EtherCAT Error] No socket connection on %s - are you running run.sh?\n", if_name);
    }
    return 0;
}

static int err_count = 0;
static int err_iteration_count = 0;
/**@brief EtherCAT errors are measured over this period of loop iterations */
#define K_ETHERCAT_ERR_PERIOD 100

/**@brief Maximum number of etherCAT errors before a fault per period of loop iterations */
#define K_ETHERCAT_ERR_MAX 20

static OSAL_THREAD_FUNC ethercat_check(const void *ptr) {
    (void) ptr;
    int slave = 0;
    while (true) {
        // count errors
        if (err_iteration_count > K_ETHERCAT_ERR_PERIOD) {
            err_iteration_count = 0;
            err_count = 0;
        }

        if (err_count > K_ETHERCAT_ERR_MAX) {
            // possibly shut down
            printf("[EtherCAT Error] EtherCAT connection degraded.\n");
            printf("[Simulink-Linux] Shutting down....\n");
            degraded_handler();
            break;
        }
        err_iteration_count++;

        if (inOP && ((wkc < expectedWKC) || ec_group[current_group].docheckstate)) {
            if (need_lf) {
                need_lf = FALSE;
                printf("\n");
            }
            /* one or more slaves are not responding */
            ec_group[current_group].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++) {
                if ((ec_slave[slave].group == current_group) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
                    ec_group[current_group].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                        printf("[EtherCAT Error] Slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                        err_count++;
                    } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
                        printf("[EtherCAT Error] Slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                        err_count++;
                    } else if (ec_slave[slave].state > 0) {
                        if (ec_reconfig_slave(slave, EC_TIMEOUT_MON)) {
                            ec_slave[slave].islost = FALSE;
                            printf("[EtherCAT Status] Slave %d reconfigured\n", slave);
                        }
                    } else if (!ec_slave[slave].islost) {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state) {
                            ec_slave[slave].islost = TRUE;
                            printf("[EtherCAT Error] Slave %d lost\n", slave);
                            err_count++;
                        }
                    }
                }
                if (ec_slave[slave].islost) {
                    if (!ec_slave[slave].state) {
                        if (ec_recover_slave(slave, EC_TIMEOUT_MON)) {
                            ec_slave[slave].islost = FALSE;
                            printf("[EtherCAT Status] Slave %d recovered\n", slave);
                        }
                    } else {
                        ec_slave[slave].islost = FALSE;
                        printf("[EtherCAT Status] Slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[current_group].docheckstate)
                printf("[EtherCAT Status] All slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(50000);
    }
}

void EtherCAT_Init(const char *if_name) {
    int i;
    int rc = 0;
    printf("[EtherCAT] Initializing EtherCAT\n");
    osal_thread_create((void *) &checkThread, 128000, (void *) &ethercat_check, (void *) &ctime);
    for (i = 1; i < 10; i++) {
        printf("[EtherCAT] Attempting to start EtherCAT, try %d of 10.\n", i);
        rc = run_ethercat(if_name);
        if (rc)
            break;
        osal_usleep(1000000);
    }
    if (rc)
        printf("[EtherCAT] EtherCAT successfully initialized on attempt %d \n", i);
    else {
        printf("[EtherCAT Error] Failed to initialize EtherCAT after 100 tries. \n");
    }
}

void EtherCAT_Transmit(EtherCAT_Msg *MasterCommand) {
    for (int i = 0; i < ec_slavecount; i++) {
        memcpy((void *) (ec_slave[0].outputs + i * sizeof(EtherCAT_Msg)), (void *) &(MasterCommand[i]),
               sizeof(EtherCAT_Msg));
    }
    ec_send_processdata();
}

static int wkc_err_count = 0;
static int wkc_err_iteration_count = 0;

//数组大小根据从站数量确定
EtherCAT_Msg Rx_Message[SLAVE_NUMBER];
EtherCAT_Msg Tx_Message[SLAVE_NUMBER];

/**
 * @description:Ethercat运行的线程函数
 * @return {*}
 */
void EtherCAT_Run() {
    if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD) {
        wkc_err_count = 0;
        wkc_err_iteration_count = 0;
    }
    if (wkc_err_count > K_ETHERCAT_ERR_MAX) {
        printf("[EtherCAT Error] Error count too high!\n");
        degraded_handler();
    }
    // send
    EtherCAT_Command_Set();
    ec_send_processdata();
    // receive
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    EtherCAT_Data_Get();
    //  check for dropped packet
    if (wkc < expectedWKC) {
        printf("\x1b[31m[EtherCAT Error] Dropped packet (Bad WKC!)\x1b[0m\n");
        wkc_err_count++;
    } else {
        need_lf = TRUE;
    }
    wkc_err_iteration_count++;
}


/**
 * @description: slave command set
 * @return {*}
 * @author: Kx Zhang
 */

void EtherCAT_Command_Set() {
    std::lock_guard<std::mutex> lock(message_mutex);
    for (int slave = 0; slave < ec_slavecount; ++slave) {
        EtherCAT_Msg_ptr msg;
        if (messages[slave].pop(msg)) {
            memcpy(&Tx_Message[slave], msg.get(), sizeof(EtherCAT_Msg));
        }
        isConfig[slave] = true;
        if (auto *slave_dest = reinterpret_cast<EtherCAT_Msg *>(ec_slave[slave + 1].outputs)) {
            *slave_dest = Tx_Message[slave];
        }
    }
}

/**
 * @description: slave data get
 * @return {*}
 * @author: Kx Zhang
 */
void EtherCAT_Data_Get() {
    for (int slave_idx = 0; slave_idx < ec_slavecount; ++slave_idx) {
        uint8_t motor_ack_status[6] = {0};
        if (auto *slave_src = reinterpret_cast<EtherCAT_Msg *>(ec_slave[slave_idx + 1].inputs)) {
            Rx_Message[slave_idx] = *reinterpret_cast<EtherCAT_Msg *>(ec_slave[slave_idx + 1].inputs);
        }
        const Slave *slave = &g_slaves[slave_idx];
        RV_can_data_repack(&Rx_Message[slave_idx], comm_ack, slave, motor_ack_status);
        if (isConfig[slave_idx]) {
            isConfig[slave_idx] = false;
            // printf("slave %d msg:\n", slave);
            // Rv_Message_Print(ack_status);
            EtherCAT_Get_State(slave_idx, motor_ack_status);
        }
    }
}

// *****************************
// *****************************
// 需要在默认的力位混合控制模式下进行工作，默认是按照ack_status=1来返回值的
/*函数功能：获取EtherCAT总线上从设备的状态信息*/
void EtherCAT_Get_State(const uint8_t slave, const uint8_t *motor_ack_status) {
    const int base_index = slave * 6;
    // printf("slave %d motor state:\n", base_index);
    std::lock_guard<std::mutex> lock(motor_data_mutex);
    for (int motor_index = 0; motor_index < 6; motor_index++) {
        // printf("motor %d state: %d\n", base_index + motor_index, motor_ack_status[motor_index]);
        if (motor_ack_status[motor_index] == 1) {
            motorDate_recv[base_index + motor_index].pos_ = rv_motor_msg[motor_index].angle_actual_rad;
            motorDate_recv[base_index + motor_index].vel_ = rv_motor_msg[motor_index].speed_actual_rad;
            motorDate_recv[base_index + motor_index].tau_ = rv_motor_msg[motor_index].current_actual_float;
            //在读取那边我已经将电流current_actual_float乘了一个KT系数，将其转化为了力矩

            motorDate_recv[base_index + motor_index].error_ = rv_motor_msg[motor_index].error;
            motorDate_recv[base_index + motor_index].temperature_ = rv_motor_msg[motor_index].temperature;
            motorDate_recv[base_index + motor_index].mos_temperature_ = rv_motor_msg[motor_index].mos_temperature;
        }
    }
}

/**
 * @description: 获取电机的数据
 * @param mot_data: 存放电机数据
 * @return {*}
 * @author: Li jinzhe
 */
void User_Get_Motor_Data(YKSMotorData *mot_data) {
    std::lock_guard<std::mutex> lock(motor_data_mutex);
    // memcpy(mot_data, motorDate_recv, sizeof(motorDate_recv));
    for (int i = 0; i < TOTAL_MOTOR_NUMBER; ++i) {
        mot_data[i].pos_ = motorDate_recv[i].pos_;
        mot_data[i].vel_ = motorDate_recv[i].vel_;
        mot_data[i].tau_ = motorDate_recv[i].tau_;

        mot_data[i].error_ = motorDate_recv[i].error_;
        mot_data[i].temperature_ = motorDate_recv[i].temperature_;
        mot_data[i].mos_temperature_ = motorDate_recv[i].mos_temperature_;
    }
}

void EtherCAT_Send_Command(const YKSMotorData *mot_data) {
    if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD) {
        wkc_err_count = 0;
        wkc_err_iteration_count = 0;
    }
    if (wkc_err_count > K_ETHERCAT_ERR_MAX) {
        printf("[EtherCAT Error] Error count too high!\n");
        degraded_handler();
    } {
        std::lock_guard<std::mutex> lock(message_mutex);
        for (int index = 0; index < TOTAL_MOTOR_NUMBER; index++) {
            const int slave_idx = index / 6;

            const Slave *slave = &g_slaves[slave_idx];
            const Motor *motor = &slave->motors[index % 6];

            if (motor->type == MOTOR_YKS) {
                // printf("slave %d command_id %d \n", slave, command_id);
                if (mot_data[index].mode == 0) {
                    send_motor_ctrl_cmd(&Tx_Message[slave_idx], motor->motor_id, motor->global_id, mot_data[index].kp_,
                                        mot_data[index].kd_, mot_data[index].pos_des_, mot_data[index].vel_des_,
                                        mot_data[index].ff_);
                } else if (mot_data[index].mode == 1) {
                    set_motor_position(&Tx_Message[slave_idx], motor->motor_id, motor->global_id,
                                       mot_data[index].pos_des_, mot_data[index].vel_des_,
                                       mot_data[index].ff_, 1);
                } else if (mot_data[index].mode == 3) {
                    set_motor_speed(&Tx_Message[slave_idx], motor->motor_id, motor->global_id, mot_data[index].vel_des_,
                                    mot_data[index].ff_, 1);
                    // printf("mode %d motor %d mode 3 \n", mot_data[index].mode, index);
                } else if (mot_data[index].mode == 2) {
                    set_motor_cur_tor(&Tx_Message[slave_idx], motor->motor_id, motor->global_id,
                                      mot_data[index].ff_, 0, 1);
                }
            } else if (motor->type == MOTOR_TI5) {
                // printf("slave %d index %d ,mode %d,pos_des_ %f ,vel_des_ %f,ff_ %f \n", slave_idx, index,
                // mot_data[index].mode, mot_data[index].pos_des_, mot_data[index].vel_des_, mot_data[index].ff_);
                if (mot_data[index].mode == 2) {
                    set_ti5_current(&Tx_Message[slave_idx], motor->motor_id, motor->global_id, mot_data[index].ff_);
                } else if (mot_data[index].mode == 3) {
                    set_ti5_speed(&Tx_Message[slave_idx], motor->motor_id, motor->global_id, mot_data[index].vel_des_);
                } else if (mot_data[index].mode == 1) {
                    set_ti5_position(&Tx_Message[slave_idx], motor->motor_id, motor->global_id,
                                     mot_data[index].pos_des_);
                }
                // printf("slave_idx %d  \n", index);
            }
        }
    }
}

void runImpl() {
    while (running) {
        EtherCAT_Run();
        usleep(1000);
    }
}

void startRun() {
    running = true;
    runThread = std::thread(runImpl);
}
