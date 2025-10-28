Deep Neural Network Deployment
===

> Dknt 2024.12

# 1 ONNX Runtime

Install pre-built library from [Github page](https://github.com/microsoft/onnxruntime).

GPU inference example code:

```cpp
#include <onnxruntime_cxx_api.h>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::string model_path =
    "/home/dknt/Project/dl_deploy/dknt_test/model/model_with_softmax.onnx";
std::string img_path = "/home/dknt/Project/dl_deploy/dknt_test/image/test.jpg";

int main() {
  /* Create session on GPU */
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "TestGpuInference");

  // The only difference is that we use OrtCUDAProviderOptions
  Ort::SessionOptions session_options;
  OrtCUDAProviderOptions cuda_options;
  cuda_options.device_id = 0;
  session_options.AppendExecutionProvider_CUDA(cuda_options);
  Ort::Session session(env, model_path.c_str(), session_options);

  /* Prepare data */
  cv::Mat input_img;
  cv::imread(img_path).convertTo(input_img, CV_32FC3, 1.0 / 255.0);

  std::vector<cv::Mat> channels(3);
  cv::split(input_img, channels);
  for (auto& channel : channels) {
    channel = channel.reshape(1, 1);
  }
  cv::Mat input_img_chw;
  cv::hconcat(channels, input_img_chw);

  std::vector<int64_t> input_shape = {1, 3, 32, 32};
  Ort::MemoryInfo memory_info =
      Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault);
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      memory_info, input_img_chw.ptr<float>(), input_img_chw.total(),
      input_shape.data(), input_shape.size());

  // Get input/output name
  char* input_name_buf = new char[50];
  char* output_name_buf = new char[50];
  strcpy(input_name_buf,
         session.GetInputNameAllocated(0, Ort::AllocatorWithDefaultOptions())
             .get());
  strcpy(output_name_buf,
         session.GetOutputNameAllocated(0, Ort::AllocatorWithDefaultOptions())
             .get());

  /* Run inference */
  // Cuda is slow for the first time, but faster for the next times
  size_t max_iter = 10000;
  std::vector<Ort::Value> output_tensor;

  // Warm up
  output_tensor = session.Run(Ort::RunOptions{nullptr}, &input_name_buf,
                                     &input_tensor, 1, &output_name_buf, 1);

  // Inference
  std::chrono::steady_clock::time_point begin =
      std::chrono::steady_clock::now();
  for (size_t i = 0; i < max_iter; ++i) {
    output_tensor = session.Run(Ort::RunOptions{nullptr}, &input_name_buf,
                                     &input_tensor, 1, &output_name_buf, 1);
  }
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cout << "Inference time = "
            << static_cast<double>(
                   std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                         begin)
                       .count()) /
                   max_iter
            << "[us]" << std::endl;

  /* Get output */
  float* output_data = output_tensor[0].GetTensorMutableData<float>();
  for (int i = 0; i < 10; i++) {
    std::cout << i << ": " << output_data[i] << std::endl;
  }

  return 0;
}
```

Example `CMakeLists.txt`:

```cmake
TOOD
```

# 2 OpenVINO

Install pre-built OpenVINO [here](https://storage.openvinotoolkit.org/repositories/openvino/packages/).

Then install OpenCL. You can check OpenCL installation by running `clinfo`.

```shell
sudo apt-get install intel-opencl-icd clinfo
```

Before building, run the script `setupvars.sh`.

Example C++ code:

```cpp
#include <openvino/openvino.hpp>

```

Example `CMakeLists.txt`:

```cmake
set(OpenVINO_DIR "<path-to-openvino-cmake>")
find_package(OpenVINO REQUIRED)
include_directories(${OpenVINO_INCLUDE_DIRS})
target_link_libraries(target openvino::runtime)
```

## 2.1


# 3 ONNX Runtime with OpenVINO

> Use OpenVINO directly. This is really hard to build.

OpenVINO serves as one of the execution providers of Onnx Runtime. To use ONNX Runtime with OpenVINO, we should **build ORT from source**:

> ONNX Runtime version `1.18.0` with OpenVINO version: `2024.1.0` is tested on my machine. Other version pairs do not work for me, unfortunately.

```shell
./build.sh --config RelWithDebInfo --update --parallel  --use_openvino AUTO:GPU,CPU --build_shared_lib --build --cmake_extra_defines CMAKE_INSTALL_PREFIX=/home/dknt/Library/onnxruntime-openvino-1.18.0
```

Additionally, to build ONNX Runtime with both CUDA and OpenVINO, run the following command (suppose all environment variables are properly set):

```shell
./build.sh --config RelWithDebInfo --update --parallel --use_cuda --use_openvino AUTO:CPU,GPU --build_shared_lib --cudnn_home /usr/lib/x86_64-linux-gnu/ --build
```

> If you have a NPU, write `--use_openvino AUTO:CPU,GPU,NPU`.

There may be some error relating to Eigen. Change the file `onnxruntime/cmake/external/onnxruntime_external_deps.cmake`:

```cmake
# find_package(Eigen3 CONFIG)
if(Eigen3_FOUND)
  get_target_property(eigen_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
else()
  include(eigen) # FetchContent
endif()
```

Example C++ code:

```cpp
Ort::SessionOptions session_options;
std::unordered_map<std::string, std::string> options;
options["device_type"] = "GPU";
options["precision"] = "FP32";
options["enable_opencl_throttling"] = "false";
session_options.AppendExecutionProvider("OpenVINO", options);
Ort::Session session(env, model_path.c_str(), session_options);
```

Example `CMakeLists.txt`:

```cmake
TOOD
```



