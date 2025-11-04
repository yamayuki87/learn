# ===============================
# ROS2 Package Creation Makefile
# ===============================

# ROS2ディストリビューション（必要に応じて変更）
ROS_DISTRO := jazzy


# 何も指定しない場合
default:pkg

# 新しいパッケージを作成
pkg:
	@if [ -z "$(name)" ]; then \
		echo "Usage: make pkg name=<package_name> [build_type=ament_cmake|ament_python]"; \
	else \
		echo "Creating ROS2 package: $(name)"; \
		ros2 pkg create $(name) --build-type ament_cmake --dependencies rclcpp; \
		echo " Done! Created at src/$(name)"; \
	fi


.PHONY: default pkg list clean_pkg
