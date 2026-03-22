# VxlSense SDK

This directory holds the VxlSense SDK release package used by VxlROS2.

## Install

Download the latest SDK from [GitHub Releases](https://github.com/asvoxel/VxlSense/releases) and extract here:

```bash
cd VxlROS2/vxlsense-sdk/
tar xzf asvxl-sdk-x.y.z-linux-x86_64.tar.gz --strip-components=1
```

After extraction, the directory should look like:

```
vxlsense-sdk/
├── include/
│   ├── vxl.h
│   ├── vxl.hpp
│   ├── vxl_types.h
│   └── ...
├── lib/
│   ├── linux/libasvxl.a
│   └── darwin/libasvxl.a
├── VERSION
└── README.md (this file)
```

## Alternative: Custom SDK Path

If the SDK is installed elsewhere, specify the path at build time:

```bash
# Via CMake argument
colcon build --cmake-args -DVXL_SDK_DIR=/opt/vxlsense/sdk

# Via environment variable
export VXL_SDK_DIR=/opt/vxlsense/sdk
colcon build
```

Path priority: CMake arg > environment variable > this directory.
