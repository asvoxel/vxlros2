# VxlSense SDK

This directory is a placeholder for the VxlSense SDK. **It is the lowest-priority fallback** —
in normal development you don't put anything here.

## SDK discovery (in priority order)

1. **CMake argument**: `colcon build --cmake-args -DVXL_SDK_DIR=/path/to/sdk`
2. **Environment variable**: `export VXL_SDK_DIR=/path/to/sdk`
3. **Sibling `vxlsdk` project** (recommended for development):
   `../../vxlsdk/sdk/current/` — auto-detected if the symlink exists
4. **This directory** (`vxlros2/vxlsdk/`) — for users installing from a release tarball

## Option A: Build from the sibling vxlsdk project (recommended)

If you have the `vxlsdk` source repository checked out next to `vxlros2`:

```
asVoxel/
├── vxlros2/   ← this project
└── vxlsdk/    ← SDK source
```

Build the SDK once and the symlink at `vxlsdk/sdk/current/` will be picked up automatically:

```bash
cd ../vxlsdk
./scripts/release-linux.sh    # or release-macos.sh
```

After this, `colcon build` in vxlros2 finds the SDK with no extra configuration.

## Option B: Use a release tarball

If you don't have the SDK source, download a release and extract it here:

```bash
cd vxlros2/vxlsdk/
tar xzf asvxl-sdk-x.y.z-linux-x86_64.tar.gz --strip-components=1
```

After extraction the directory should contain:

```
vxlsdk/
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

Releases: https://github.com/asvoxel/VxlSense/releases
