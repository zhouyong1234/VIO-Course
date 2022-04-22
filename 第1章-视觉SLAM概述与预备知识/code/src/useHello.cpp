#include "hello.h"
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <iostream>
#include <typeinfo>

DEFINE_int32(print_times, 1, "print_times");


int main( int argc, char *argv[] ) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;
  FLAGS_alsologtostderr = true;
  FLAGS_v = 2;
  // sayHello();
  google::SetStderrLogging(google::INFO);
  google::InstallFailureSignalHandler();
  // std::cout << "argv[0]: " << argv[0] << " " << "argv[1]: " << argv[1] << std::endl;
  // std::cout << FLAGS_print_times << std::endl;
  for(int i = 0; i < FLAGS_print_times; i++)
  {
    LOG(INFO) << "Hello SLAM";
  }
  google::ShutdownGoogleLogging();
  return 0;
}
