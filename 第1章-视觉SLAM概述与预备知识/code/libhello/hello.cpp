#include "hello.h"
#include <iostream>

#include <glog/logging.h>

void sayHello()
{
    LOG(INFO) << "Hello SLAM";
}
