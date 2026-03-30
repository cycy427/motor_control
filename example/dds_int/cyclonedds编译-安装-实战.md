# 0. by mrtang 2025/4/12

# 1.提示

**终极目的**：使用c++和python来开发dds通信程序

**技术手段**：基于cyclonedds库 （这个库的c语言开发代码比较繁琐，所以不考虑用c语言，直接上c++）

**cyclonedds库的安装**：step1: 安装cyclonedds核心库，step2:安装cyclonedds的C++绑定，step3:安装cyclonedds的python绑定

**本工作基于cyclonedds-0.10.2版本（考虑稳定可靠，这不是最新版）**

# 2.准备工作

- 下载或克隆cyclonedds（核心库，注意版本一致）：[guoguomumu/cyclonedds](https://gitee.com/guoguomumu/cyclonedds.git)

- 下载或克隆cyclonedds-cxx（c++绑定，注意版本一致）：[guoguomumu/cyclonedds-cxx](https://gitee.com/guoguomumu/cyclonedds-cxx.git)

- 阅读cyclonedds-python （python绑定，注意版本一致）：[zfwt/cyclonedds-python ](https://gitee.com/flyingsuper/cyclonedds-python)
# 3.编译安装cyclonedds核心库

- 进入cyclonedds-0.10.2目录

- 创建并进入build子目录
  
  - 命令：mkdir build && cd build

- cmake,  **提示：我们把cyclonedds库安装到系统路径/usr/local，便于所有用户程序都能直接引用该库 ****
  
  - 命令：cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_IDLC=ON ..
  
  - 参数含义：指定安装目录，指定开启IDLC工具编译

- 编译和安装
  
  - 命令：sudo cmake --build . --config RelWithDebInfo --target install

- 验证安装
  
  - 命令：idlc -h (应该能看到相关信息，说明这个工具是有的)
  
  - 命令：ls /usr/local/lib/libdds\* (验证目录下是否有dds相关的库)

# 4.编译安装cyclonedds-cxx绑定

- 进入cyclonedds-cxx-0.10.2目录
  
  - 创建并进入build子目录
  
  - 命令：mkdir build && cd build

- cmake,  **提示：我们把cyclonedds库安装到系统路径/usr/local，便于所有用户程序都能直接引用该库 ****
  
  - 命令：cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_PREFIX_PATH=/usr/local -DBUILD_EXAMPLES=ON ..
  
  - 参数含义：指定安装目录/usr/local, 开启example编译

- 编译安装
  
  - sudo cmake --build . --target install

- 验证
  
  - 完成这些之后，会在build/bin/example目录下生成ddscxxHelloworldPublisher和ddscxxHelloworldSubscriber，运行他们可以看到一个发布，一个订阅，相互发送消息，通信成功
  
  - **idlc cxx工具验证**
    
    进入根目录下examples/helloworld文件夹，启用终端运行
    
    命令：idlc -l cxx HelloWroldData.idl
    
    正常会生成HelloworldData.cpp, HelloworldData.hpp两个文件，也就是将idl消息文件转换成c++文件。终端可能提示：no default extensibility provide（警告，不是报错, 在idl书写规范中可以查entisibility的含义）， 我们可以打开idl文件，修改定义：
    
    ```idl
    module HelloWorldData
    {
      @extensibility(FINAL)   # 明确申明extensibilty
      struct Msg
      {
        @key long userID;    # usrID为key字段
        string message;
      };
    };
    ```
    
    再转换时将不会提示警告。

# 5.安装python包

- 命令：cd path/to/cyclonedds-python-0.10.2，目标是到达cyclonedds-python-0.10.2目录

- 命令：conda activate 环境名 ，切换python环境

- 命令：pip3 install .

- 可能遇到报错： xxxxx could not locate cyclonedds ... (原因是找不到cyclonedds核心库)

- 解决方案：添加环境变量

- 命令： sudo gedit ~/.bashrc

- 在打开的.bashrc文件中最后一行添加： export CYCLONEDDS_HOME=/usr/local

- 保存和关闭文件后，命令：source ~/.bashrc

- 重新执行命令：pip3 install .

- 验证，打开python, import cyclonedds, 或者通过pip list查看

- **验证idlc py工具**

- 在example/helloworld文件夹下：执行命令： idlc -l py HelloWorldData.idl查看是否能成功转换idl文件为py。注意要分辨是找不到idlc工具，还是找不到pygenerator, 或者是idl语法错误

# 6.实战

- 定义消息idl (nubotmsg.idl)
  
  ```
  module NubotZ1
  {
    @extensibility(FINAL) 
    struct Msg
    {
      int32 ID;
      int32 count;
    };
  };
  ```

- 转换成py代码
  
  命令：idlc -l py nubotmsg.idl
  
  在当前目录生成NubotZ1文件夹，内部包含_nubotmsg.py

- 创建ddsdemo工程

- 编写cameklist.txt
  
  ```
    cmake_minimum_required(VERSION 3.5)
    project(ddsdemo LANGUAGES C CXX)
  
    # 加载cyclonedds-cxx库
    if(NOT TARGET CycloneDDS-CXX::ddscxx)
      find_package(CycloneDDS-CXX REQUIRED)
    endif()
  
    # 调用生成器转换idl消息，它能自动能处理nubotmsg.hpp的相关依赖
    # 如果手动转换idl消息生成cpp,hpp文件，则需要处理较为复杂的依赖关系（hpp内部还会引用cyclone库的相关头文件），因此直接使用生成器更为方便
    idlcxx_generate(TARGET nubotmsg FILES nubotmsg.idl)
  
    add_executable(cxxpub cxxpub.cpp)
    add_executable(cxxsub cxxsub.cpp)
  
    # Link both executables to idl data type library and ddscxx.
    target_link_libraries(cxxpub CycloneDDS-CXX::ddscxx nubotmsg)
    target_link_libraries(cxxsub CycloneDDS-CXX::ddscxx nubotmsg)
  
    # Disable the static analyzer in GCC to avoid crashing the GNU C++ compiler
    # on Azure Pipelines
    if(DEFINED ENV{SYSTEM_TEAMFOUNDATIONSERVERURI})
      if(CMAKE_C_COMPILER_ID STREQUAL "GNU" AND ANALYZER STREQUAL "on")
        target_compile_options(cxxpub PRIVATE -fno-analyzer)
        target_compile_options(cxxsub PRIVATE -fno-analyzer)
      endif()
    endif()
  
    set_property(TARGET cxxpub PROPERTY CXX_STANDARD 17)
    set_property(TARGET cxxsub PROPERTY CXX_STANDARD 17)
  ```

- 编写发布者代码c++ （cxxpub.cpp）
  
  ```
    #include <cstdlib>
    #include <iostream>
    #include <chrono>
    #include <thread>
    #include <csignal>
    #include <atomic>
  
    #include "dds/dds.hpp"
    #include "nubotmsg.hpp"
  
    using namespace org::eclipse::cyclonedds;
  
    //用于响应ctrl+c退出
    std::atomic<bool> quit(false);
  
    void signal_handler(int signal) {
        if (signal == SIGINT) {
            std::cout << "\nReceived Ctrl+C, exiting gracefully..." << std::endl;
            quit = true;  // 设置退出标志
        }
    }
  
    //在dds中，参与者相互通信要满足的条件
    //同一个通信域
    //同样的topic名称
    //同样的消息类型（必须严格对应，在跨语言通信时（c++ <--> py），建议使用生成器来转换idl消息，不要手动编写）
  
    int main() {
        // 注册信号处理函数
        std::signal(SIGINT, signal_handler);
  
        std::cout << "=== [Publisher] Creating participant." << std::endl;
        //创建域参与者（domain participant）,通信域id为0.
        //在dds数据分发中，参与这必须在同一个通信域中才可以通信，默认参数为0
        dds::domain::DomainParticipant participant(0);
  
        // 设置 Topic QoS, Qos:通信服务策略，简单来说就是消息以什么方式传输，是一定送达？尽量送达？等
        dds::topic::qos::TopicQos topicQos;
        topicQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
                 << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
                 << dds::core::policy::History::KeepLast(5); //保留近5条消息，对于BestEffort来说应该没用（mrtang）
  
        //创建topic
        dds::topic::Topic<NubotZ1::Msg> topic(participant, "/nubot/z1/testtopic", topicQos);
  
        // 创建 Publisher 和 DataWriter
        dds::pub::Publisher publisher(participant);
        dds::pub::qos::DataWriterQos writerQos(topicQos);  // datawriter的qos应当继承自topic的qos
        dds::pub::DataWriter<NubotZ1::Msg> writer(publisher, topic, writerQos);
  
        int i = 0;
        while (!quit) {
            NubotZ1::Msg msg(0, i++);
            std::cout << "=== [Publisher] Writing sample " << i << std::endl;
            writer.write(msg);  //发布消息
  
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
  
        std::cout << "=== [Publisher] Exiting." << std::endl;
        return EXIT_SUCCESS;
    }
  ```

- 编写订阅者代码c++ (cxxsub.cpp)
  
  ```
  #include <cstdlib>
  #include <iostream>
  #include <chrono>
  #include <thread>
  #include <csignal>
  #include <atomic>
  
  #include "dds/dds.hpp"
  #include "nubotmsg.hpp"
  
  using namespace org::eclipse::cyclonedds;
  
  //用于响应ctrl+c退出
  std::atomic<bool> quit(false);
  
  void signal_handler(int signal) {
      if (signal == SIGINT) {
          std::cout << "\nReceived Ctrl+C, exiting gracefully..." << std::endl;
          quit = true;  // 设置退出标志
      }
  }
  
  int main() {
      // 注册信号处理函数
      std::signal(SIGINT, signal_handler);
  
      std::cout << "=== [Subscriber] Create reader." << std::endl;
      dds::domain::DomainParticipant participant(0);
      dds::topic::Topic<NubotZ1::Msg> topic(participant, "/nubot/z1/testtopic");
      dds::sub::Subscriber subscriber(participant);
      dds::sub::DataReader<NubotZ1::Msg> reader(subscriber, topic);
  
      std::cout << "=== [Subscriber] Wait for message." << std::endl;
      while (!quit) {
          dds::sub::LoanedSamples<NubotZ1::Msg> samples;
          samples = reader.take();
          if (samples.length() > 0) {
              dds::sub::LoanedSamples<NubotZ1::Msg>::const_iterator sample_iter;
              for (sample_iter = samples.begin();sample_iter < samples.end();++sample_iter) {
                  const NubotZ1::Msg& msg = sample_iter->data();
                  const dds::sub::SampleInfo& info = sample_iter->info();
                  if (info.valid()) {
                      std::cout << "=== [Subscriber] Message received:";
                      std::cout << "    count  : " << msg.count() << std::endl;
                  }
              }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      return EXIT_FAILURE;
  }
  ```

- 编写发布者代码python (pub.py)
  
  ```
  import time
  from cyclonedds.domain import DomainParticipant
  from cyclonedds.pub import Publisher, DataWriter
  from cyclonedds.topic import Topic
  from cyclonedds.sub import DataReader
  from cyclonedds.qos import Qos, Policy
  from dataclasses import dataclass
  from cyclonedds.idl import IdlStruct
  import cyclonedds.idl.types as types
  
  import cyclonedds.idl as idl
  import cyclonedds.idl.annotations as annotate
  import cyclonedds.idl.types as types
  
  # 这是我从idl转换得到的py文件中复制过来的
  @dataclass
  @annotate.final
  @annotate.autoid("sequential")
  class Msg(idl.IdlStruct, typename="NubotZ1.Msg"):
      ID: types.int32
      count: types.int32
  
  def main():
      # 创建域参与者
      participant = DomainParticipant(0)
  
      # 创建主题
      topic = Topic(participant, "/nubot/z1/testtopic", Msg)
  
      # 创建 QoS 策略
      qos = Qos(
          Policy.Reliability.BestEffort,  # 或 Policy.Reliability.Reliable
          Policy.Durability.Volatile,     # 或 Policy.Durability.TransientLocal
          Policy.History.KeepLast(5),    # 保留最后10条消息
      )
  
      # 创建发布者和数据写入器
      publisher = Publisher(participant)
      writer = DataWriter(publisher, topic, qos=qos)
  
      # 发布消息
      i = 0
      while True:
          i+=1
          message = Msg(1,i)
          print(f"Writing message: {message.ID}, count: {message.count}")
          writer.write(message)
          time.sleep(1)
  
  if __name__ == '__main__':
      main()
  ```

- 编写订阅者代码python (sub.py)
  
  ```
  import signal
  import time
  from cyclonedds.domain import DomainParticipant
  from cyclonedds.topic import Topic
  from cyclonedds.sub import DataReader
  from cyclonedds.qos import Qos, Policy
  from dataclasses import dataclass
  from cyclonedds.idl import IdlStruct
  import numpy as np
  import ctypes
  import cyclonedds.idl.types as types
  
  import cyclonedds.idl as idl
  import cyclonedds.idl.annotations as annotate
  import cyclonedds.idl.types as types
  
  # 这是我从idl转换得到的py文件中复制过来的
  @dataclass
  @annotate.final
  @annotate.autoid("sequential")
  class Msg(idl.IdlStruct, typename="NubotZ1.Msg"):
      ID: types.int32
      count: types.int32
  
  def main():
      print("=== [Subscriber] Creating participant ===")
      participant = DomainParticipant(domain_id=0)  # 默认domain_id=0
  
      # 设置QoS（与发布端一致）
      topic_qos = Qos(
          Policy.Reliability.BestEffort,
          Policy.Durability.Volatile,
          Policy.History.KeepLast(5)
      )
  
      # 创建Topic
      topic = Topic(participant, "/nubot/z1/testtopic", Msg, qos=topic_qos)
  
      # 创建Reader
      reader = DataReader(participant, topic)
  
      print("=== [Subscriber] Ready to receive messages ===")
      while True:
          # 非阻塞读取（timeout=0.1秒）
          for msg in reader.take():
              print(f"Received: ID={msg.ID}, count={msg.count}")
  
          time.sleep(0.1)
  
      print("=== [Subscriber] Exiting cleanly ===")
      return 0
  
  if __name__ == "__main__":
      main()
  ```

# 7.删除cyclonedds (重装时需要）

   cyclonedds的相关文件安装目录包括（本例<intall-location>为/usr/local)：

  < install-location>/lib

  < install-location>/bin

  < install-location>/include

  < install-location>share



   可通过如下关键字删除（例如sudo rm -rf libcyclone*）: libcyclone, libdds, dds, idlc, Cyclone 

# 8. Enjoy
