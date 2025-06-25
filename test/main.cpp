#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <cstdlib>
#include <cstring>  // 添加此行以支持 std::strlen

volatile sig_atomic_t stop_flag = 0;

void signalHandler(int signal) {
    if (signal == SIGINT) {
        stop_flag = true;
        const char *msg = "\nSIGINT received, preparing to exit...\n";
        write(STDOUT_FILENO, msg, std::strlen(msg));  // 现在可以正确找到 strlen
    }
}

int main() {
    signal(SIGINT, signalHandler);

    std::cout << "Program is running. Press Ctrl+C to exit." << std::endl;

    while (!stop_flag) {
        std::cout << "hello" << std::endl;
        sleep(1);
    }

    std::cout << "Exiting program." << std::endl;
    return 0;
}

