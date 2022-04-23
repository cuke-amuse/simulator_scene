## 介绍

4月23日 可视化且动态配置Apollo的规划集成测试

## 修改
将测试程序从GTest 方式修改为可执行程序，以独立运行且动态接收外部的参数

## 使用方法

编译：bazel test modules/planning/integration_tests:sunnyvale_big_loop_test_local
运行：bazel-bin/modules/planning/integration_tests/sunnyvale_big_loop_test_local -i sunnyvale_big_loop_test -n 400 -t 0
