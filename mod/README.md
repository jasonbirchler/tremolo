# FunBox libDaisy patch

`daisy_petal.h` and `daisy_petal.cpp` together define the FunBox hardware patch. These files are the source of truth; the top-level build synchronizes them into `libDaisy/src` before rebuilding the libraries.

From the repository root:

```sh
make patch-libdaisy
```

To synchronize the patch and build both libraries:

```sh
make libs
```

The normal `make` build performs both steps before building the firmware.
