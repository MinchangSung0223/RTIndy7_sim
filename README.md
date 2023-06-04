# RTIndy7_sim

Lie Robotics = LR

Modern Robotics = MR

1.build
```bash
cd sim
mkdir build
cd build
cmake ..
make -j16
```
2.start
```bash
   ./${EXCUTABLE FILES}
```

+ How to make MR_info.json or LR_info.json
```bash
   cd urdf_loader
   python3 urdf_to_MR.py ${urdf_path}
   or
   python3 urdf_to_LR.py ${urdf_path}
```

