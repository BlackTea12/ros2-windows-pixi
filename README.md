# (pixi) windows ros2 environments

### 1. Prequisites

- [Visual Studio 2022](https://aka.ms/vs/17/release/vs_community.exe)
- [pixi](https://pixi.prefix.dev/latest/installation/)

### 2. Environment depenencies setups

Please refer to links below for your preferences.

- [jazzy prefixs](https://prefix.dev/channels/robostack-jazzy)
- [jazzy available packages](https://robostack.github.io/jazzy.html)

### 3. Install environments

``` bash
pixi install
pixi shell -e jazzy # activate jazzy env
pixi config set --local run-post-link-scripts insecure # (optional)
```

### 4. Use ros2 commands

Just like you use package create command in linux, open your command prompt in windows and type the same command.


``` bash
ros2 pkg create --node-name {node_name} --library-name {library_name} --build-type ament_cmake --license {license} {package_name}
```

If you want to test pre-created package, go see `colcon_ws/src/spin_controller`.

As `[tasks.build]` command is added, you can build package like this.

``` bash
pixi run build
```

Or just use `colcon build` like always :blush: