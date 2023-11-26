## CASADI install instructions for C++ with ROS2

### Installing Casadi

- Follow the link instructions here [Linux Install Instructions](https://github.com/casadi/casadi/wiki/InstallationLinux)
- For a Docker based installation, add the same instructions into your Dockerfile with the RUN command
- Make sure to add the dependencies required into the Dockerfile commands as well
- Once this is done, verify by opening Python and trying `import casadi`

### Using Casadi with C++

- Use `#include <casadi/casadi.hpp>` to import Casadi into the cpp files
- The primary datatype is the `casadi::MX` datatype - just set everything to that
- For non-linear optimization problems, the Opti stack section of the [Docs](https://web.casadi.org/docs/) is quite helpful

### Casadi with ROS2

By default, Casadi is not identified as a CMake package, so a few steps are needed to get it working with ROS2

- In your package directory, cfreate a new folder called `cmake_modules`
- In the source files cloned from the casadi git, copy the file `casadi/cmake/FindCASADI.cmake` into the created directory
- For docker, do this on the host PC and then when you mount it, the file should be available in the docker image

Now to modify the `CMakeLists.txt`:

- Add this to a line before the 'foreach' statement -> ` set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)`

- Add 'CASADI' to the `foreach` list

- To `include_directories`, add `${CASADI_INCLUDE_DIR}` to the list

- After the `ament_target_dependencies` line, add the following segment to link the target libraries

    ```
    target_link_libraries(
    mpc_node
    ${CASADI_LIBRARIES}
    )
    ```

