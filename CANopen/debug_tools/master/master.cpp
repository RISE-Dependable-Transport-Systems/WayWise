#include <lely/ev/loop.hpp>
#if _WIN32
#include <lely/io2/win32/ixxat.hpp>
#include <lely/io2/win32/poll.hpp>
#elif defined(__linux__)
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#else
#error This file requires Windows or Linux.
#endif
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>

#include <iostream>

#include <iostream>
#if _WIN32
#include <thread>
#endif

using namespace std::chrono_literals;
using namespace lely;

// This driver inherits from FiberDriver, which means that all CANopen event
// callbacks, such as OnBoot, run as a task inside a "fiber" (or stackful
// coroutine).
class MyDriver : public canopen::FiberDriver {
 public:
  using FiberDriver::FiberDriver;

 private:
  // This function gets called when the boot-up process of the slave completes.
  // The 'st' parameter contains the last known NMT state of the slave
  // (typically pre-operational), 'es' the error code (0 on success), and 'what'
  // a description of the error, if any.
  void
  OnBoot(canopen::NmtState /*st*/, char es,
         const std::string& what) noexcept override {
    if (!es || es == 'L') {
      std::cout << "slave " << static_cast<int>(id()) << " booted sucessfully"
                << std::endl;
    } else {
      std::cout << "slave " << static_cast<int>(id())
                << " failed to boot: " << what << std::endl;
    }
  }

  // This function gets called during the boot-up process for the slave. The
  // 'res' parameter is the function that MUST be invoked when the configuration
  // is complete. Because this function runs as a task inside a coroutine, it
  // can suspend itself and wait for an asynchronous function, such as an SDO
  // request, to complete.
  void
  OnConfig(std::function<void(std::error_code ec)> res) noexcept override {
    try {
      // Perform a few SDO write requests to configure the slave. The
      // AsyncWrite() function returns a future which becomes ready once the
      // request completes, and the Wait() function suspends the coroutine for
      // this task until the future is ready.

      // Configure the slave to monitor the heartbeat of the master (node-ID 1)
      // with a timeout of 2000 ms.
      Wait(AsyncWrite<uint32_t>(0x1016, 1, (1 << 16) | 2000));
      // Configure the slave to produce a heartbeat every 1000 ms.
      Wait(AsyncWrite<uint16_t>(0x1017, 0, 1000));
      // Configure the heartbeat consumer on the master.
      ConfigHeartbeat(2000ms);

      // Reset object 4000:00 and 4001:00 on the slave to 0.
      //Wait(AsyncWrite<uint32_t>(0x4000, 0, 0));
      //Wait(AsyncWrite<uint32_t>(0x4001, 0, 0));
      Wait(AsyncWrite<int8_t>(0x2000, 1, 0));
      Wait(AsyncWrite<int16_t>(0x2000, 2, 0));
      Wait(AsyncWrite<uint8_t>(0x2000, 3, 0));
      //Wait(AsyncWrite<uint8_t>(0x2000, 6, 1));
      Wait(AsyncWrite<uint8_t>(0x2001, 1, 0));
      Wait(AsyncWrite<int8_t>(0x2001, 2, 0));
      Wait(AsyncWrite<int16_t>(0x2001, 3, 0));
      Wait(AsyncWrite<uint8_t>(0x2001, 4, 80));
      Wait(AsyncWrite<uint16_t>(0x2001, 5, 600));

      // Report success (empty error code).i
      res({});
    } catch (canopen::SdoError& e) {
      // If one of the SDO requests resulted in an error, abort the
      // configuration and report the error code.
      res(e.code());
    }
  }

  // This function is similar to OnConfg(), but it gets called by the
  // AsyncDeconfig() method of the master.
  void
  OnDeconfig(std::function<void(std::error_code ec)> res) noexcept override {
    try {
      // Disable the heartbeat consumer on the master.
      ConfigHeartbeat(0ms);
      // Disable the heartbeat producer on the slave.
      Wait(AsyncWrite<uint16_t>(0x1017, 0, 0));
      // Disable the heartbeat consumer on the slave.
      Wait(AsyncWrite<uint32_t>(0x1016, 1, 0));
      res({});
    } catch (canopen::SdoError& e) {
      res(e.code());
    }
  }

  // This function gets called every time a value is written to the local object
  // dictionary of the master by an RPDO (or SDO, but that is unlikely for a
  // master), *and* the object has a known mapping to an object on the slave for
  // which this class is the driver. The 'idx' and 'subidx' parameters are the
  // object index and sub-index of the object on the slave, not the local object
  // dictionary of the master.
  /**
 * Fractional values are exchanged via CAN communication, but the CAN protocol is based on using integer numbers.
 * Using y=k*x+m, convertion to and from int is done.
 * Convert to sensor measurement value:  value = msgData*resolution + offset
 *
 * @param value
 * The sensor measurement value
 *
 * @param resolution
 * The resolution of your measurements
 *
 * @param offset
 * If your sensor interval has an offset, e.g 100 if [100, 300]
 *
 * @param msgData
 * The data sent over CAN
 */
  void
  OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override {
/*    if (idx == 0x4001 && subidx == 0) {
      // Obtain the value sent by PDO from object 4001:00 on the slave.
      uint32_t val = rpdo_mapped[0x4001][0];
      std::cout << "Mottaget (4001:00) " << val << std::endl;
      // Increment the value and store it to an object in the local object
      // dictionary that will be sent by TPDO to object 4000:00 on the slave.
      tpdo_mapped[0x4000][0] = ++val;
    }*/
    if (idx == 0x2000 && subidx == 1) {
      // Obtain the value sent by PDO from object 2000:02 on the slave.
      int8_t val = rpdo_mapped[0x2000][1];
      std::cout << "Mottaget (2000:01) " << (int16_t)val << std::endl;
      // Increment the value and store it to an object in the local object
      // dictionary that will be sent by TPDO to object 2001:02 on the slave.
      tpdo_mapped[0x2001][2] = val++;
    }
    if (idx == 0x2000 && subidx == 2) {
      // Obtain the value sent by PDO from object 2000:02 on the slave.
      int16_t val = rpdo_mapped[0x2000][2];
      double resolution = 0.0001;
      std::cout << "Mottaget (2000:02) " << val << std::endl;
      
      std::cout << "Mottaget (2000:02) value: " << val*resolution << std::endl;
      // Increment the value and store it to an object in the local object
      // dictionary that will be sent by TPDO to object 2001:03 on the slave.
      tpdo_mapped[0x2001][3] = val++;
    }
    if (idx == 0x2000 && subidx == 3) {
      // Obtain the value sent by PDO from object 2000:03 on the slave.
      uint8_t val = rpdo_mapped[0x2000][3];
      std::cout << "Mottaget (2000:03) " << (int16_t)val << std::endl;
      // Increment the value and store it to an object in the local object
      // dictionary that will be sent by TPDO to object 2001:01 on the slave.
      tpdo_mapped[0x2001][1] = val; //(uint8_t)2; //Controls the status
    }
    if (idx == 0x2000 && subidx == 4) {
      // Obtain the value sent by PDO from object 2000:04 on the slave.
      int16_t val = rpdo_mapped[0x2000][4];
      std::cout << "Mottaget (2000:04) " << val << std::endl;
      tpdo_mapped[0x2001][6] = val;
    }
    if (idx == 0x2000 && subidx == 5) {
      // Obtain the value sent by PDO from object 2000:05 on the slave.
      int32_t val = rpdo_mapped[0x2000][5];
      std::cout << "Mottaget (2000:05) " << val << std::endl;
      tpdo_mapped[0x2001][7] = val;
    }
    if (idx == 0x2000 && subidx == 6) {
      // Obtain the value sent by PDO from object 2000:06 on the slave.
      uint32_t val = rpdo_mapped[0x2000][6];
      std::cout << "Mottaget (2000:06) " << (int64_t)val << std::endl;
    }
     if (idx == 0x2002 && subidx == 1) {
      // Obtain the value sent by PDO from object 2000:06 on the slave.
      int16_t val = rpdo_mapped[0x2002][1];
      std::cout << "Mottaget (2002:01) " << val << std::endl;
    }
     if (idx == 0x2002 && subidx == 2) {
      // Obtain the value sent by PDO from object 2000:06 on the slave.
      int16_t val = rpdo_mapped[0x2002][2];
      std::cout << "Mottaget (2002:02) " << val << std::endl;
    }
     if (idx == 0x2002 && subidx == 3) {
      // Obtain the value sent by PDO from object 2000:06 on the slave.
      int16_t val = rpdo_mapped[0x2002][3];
      std::cout << "Mottaget (2002:03) " << val << std::endl;
    }
     if (idx == 0x2002 && subidx == 4) {
      // Obtain the value sent by PDO from object 2000:06 on the slave.
      int8_t val = rpdo_mapped[0x2002][4];
      std::cout << "Mottaget (2002:04) " << (int16_t)val << std::endl;
    }
  }
};

int
main() {
  // Initialize the I/O library. This is required on Windows, but a no-op on
  // Linux (for now).
  io::IoGuard io_guard;
#if _WIN32
  // Load vcinpl2.dll (or vcinpl.dll if CAN FD is disabled).
  io::IxxatGuard ixxat_guard;
#endif
  // Create an I/O context to synchronize I/O services during shutdown.
  io::Context ctx;
  // Create an platform-specific I/O polling instance to monitor the CAN bus, as
  // well as timers and signals.
  io::Poll poll(ctx);
  // Create a polling event loop and pass it the platform-independent polling
  // interface. If no tasks are pending, the event loop will poll for I/O
  // events.
  ev::Loop loop(poll.get_poll());
  // I/O devices only need access to the executor interface of the event loop.
  auto exec = loop.get_executor();
  // Create a timer using a monotonic clock, i.e., a clock that is not affected
  // by discontinuous jumps in the system time.
  io::Timer timer(poll, exec, CLOCK_MONOTONIC);
#if _WIN32
  // Create an IXXAT CAN controller and channel. The VCI requires us to
  // explicitly specify the bitrate and restart the controller.
  io::IxxatController ctrl(0, 0, io::CanBusFlag::NONE, 125000);
  ctrl.restart();
  io::IxxatChannel chan(ctx, exec);
#elif defined(__linux__)
  // Create a virtual SocketCAN CAN controller and channel, and do not modify
  // the current CAN bus state or bitrate.
  //io::CanController ctrl("vcan0");
  io::CanController ctrl("can0");
  io::CanChannel chan(poll, exec);
#endif
  chan.open(ctrl);

  // Create a CANopen master with node-ID 1. The master is asynchronous, which
  // means every user-defined callback for a CANopen event will be posted as a
  // task on the event loop, instead of being invoked during the event
  // processing by the stack.
  canopen::AsyncMaster master(timer, chan, "master.dcf", "", 1);

  // Create a driver for the slaves with node-ID 2 and 3. Just add a driver for each new slave.
  MyDriver driver(exec, master, 2);
  //MyDriver driver2(exec, master,3);

  // Create a signal handler.
  io::SignalSet sigset(poll, exec);
  // Watch for Ctrl+C or process termination.
  sigset.insert(SIGHUP);
  sigset.insert(SIGINT);
  sigset.insert(SIGTERM);

  // Submit a task to be executed when a signal is raised. We don't care which.
  sigset.submit_wait([&](int /*signo*/) {
    // If the signal is raised again, terminate immediately.
    sigset.clear();
    // Tell the master to start the deconfiguration process for all nodes, and
    // submit a task to be executed once that process completes.
    master.AsyncDeconfig().submit(exec, [&]() {
      // Perform a clean shutdown.
      ctx.shutdown();
    });
  });

  // Start the NMT service of the master by pretending to receive a 'reset
  // node' command.
  master.Reset();

#if _WIN32
  // Create two worker threads to ensure the blocking canChannelReadMessage()
  // and canChannelSendMessage() used by the IXXAT CAN channel do not hold up
  // the event loop.
  std::thread workers[] = {std::thread([&]() { loop.run(); }),
                           std::thread([&]() { loop.run(); })};
#endif

  // Run the event loop until no tasks remain (or the I/O context is shut down).
  loop.run();

#if _WIN32
  // Wait for the worker threads to finish.
  for (auto& worker : workers) worker.join();
#endif

  return 0;
}
