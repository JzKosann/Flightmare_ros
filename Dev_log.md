# Dev_log

## 2025.3.6

- **『ROS问题』**rosrun python 文件的时候，指定不同的python编译器
  - 由于同一系统下python过多，且python脚本可自行执行，可指定编译器执行，例如：
    `/usr/bin/python3 XXX/XXX/&&&.py`

## 2025.3.8

- 『添加了单电机的推力控制』：

  

  ```c++
  Scalar t = 0.0;
  Vector<4> thrusts{500, 500, 500, 500};
  Command cmd(t, thrusts);
  Scalar ctl_dt = 0.01;
      // 检查命令是否有效
  if (cmd.valid())
  {
      std::cout << "Command is valid." << std::endl;
  }
  	// 检查控制模式
  if (cmd.isSingleRotorThrusts())
  {
      std::cout << "Using single rotor thrusts mode." << std::endl;
      std::cout << "Thrusts: " << cmd.thrusts.transpose() << std::endl;
  }
  quad_ptr->getDynamics();
  quad_ptr->run(cmd, ctl_dt);
  ```

- 待解决问题....
  - 『NMPC控制框架』：单电机输出应该从转速到推力转换
  - 『DYNAMICS 方程』：在 `flightlib` 中找一下

## 2025.3.9

-  完善了Dynamic 方程和 `model.py`，随后需要进行测试，

  参考代码：https://github.com/LGQWakkk/Quadrotor-NMPC-Control/blob/main/export_model.py
