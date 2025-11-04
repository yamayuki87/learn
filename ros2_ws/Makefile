
default: cb

# 全体ビルド
alcb:
	colcon build --mixin ninja ccache compile-commands lld --symlink-install;

#パッケージセレクトビルド
#-zは空文字チェック
#pkgはcbの後ろに指定する
# @がないとコマンドがターミナルに表示される
cb:
	@if [ -z "$(pkg)" ]; then \
		echo "Usage: make cb pkg=<package_name>"; \
	else \
		colcon build --mixin ninja ccache compile-commands lld --symlink-install --packages-select $(pkg); \
	fi

# セットアップスクリプトを読み込む
so:
	@bash -c "source install/setup.bash"
	@echo "Workspace sourced!"

deps:
	@echo "Installing dependencies via rosdep..."
	sudo apt update
	rosdep update
	rosdep install -i --from-path src --rosdistro jazzy -y
	@echo "Dependencies installed!"


# クリーン
cle:
	rm -rf build install log
	@echo "Workspace cleaned!"

# 実行（パッケージとノード名を指定）
run:
	@if [ -z "$(pkg)" ] || [ -z "$(node)" ]; then \
		echo "Usage: make run pkg=<package_name> node=<node_name>"; \
	else \
		source install/setup.bash && ros2 run $(pkg) $(node); \
	fi

launch:
	@if [ -z "$(pkg)" ] || [ -z "$(file)" ]; then \
		echo "Usage: make launch pkg=<package_name> file=<launch_file>"; \
	else \
		source install/setup.bash && ros2 launch $(pkg) $(file); \
	fi
# テスト（例：ros2 test）
test:
	source install/setup.bash && colcon test

lis:
	colcon list


.PHONY: default build source clean run test info

#cb so cle run launch test lis deps
#source: so
#list: lis
#deps: rosdep
