# Sleipnir motion profile benchmark

This repository runs benchmarks of motion profiles using Sleipnir.

## Build instructions

### Dependencies

Install the following packages: cmake and gcc.

### Build and run

```bash
cmake -B build -S .
cmake --build build
./build/SleipnirBench
```

# Results

## roboRIO

Ran via ./SleipnirBench --benchmark_time_unit=us

```
Running ./SleipnirBench
Run on (2 X 866.66 MHz CPU s)
Load Average: 1.45, 1.29, 1.14
----------------------------------------------------------------------------------------
Benchmark                                              Time             CPU   Iterations
----------------------------------------------------------------------------------------
Sleipnir_MotionProfile_TrapezoidTrapezoid            568 us          558 us         1270
Sleipnir_MotionProfile_ExponentialExponential       1806 us         1779 us          393
Sleipnir_MotionProfile_TrapezoidExponential          447 us          441 us         1587
Sleipnir_MotionProfile_ExponentialTrapezoid         1257 us         1236 us          566
```
