# Download and install the tools used for local builds and GitHub CI.
#
# The Arm compiler is the xPack GNU Arm Embedded GCC release pinned by
# XPACK_GCC_VER in make/tools.mk. Archives come from GitHub Releases so CI
# does not depend on a separately hosted tools tarball staying in sync with
# the path in tools.mk.
#
# Windows additionally still pulls windows-tools.zip for the make/ utilities
# used by the Windows CI job (tools/windows/make/bin/make).

# Shared pin (must match make/tools.mk)
XPACK_GCC_VER ?= 15.2.1-1.1
XPACK_GCC_DIR ?= xpack-arm-none-eabi-gcc-$(XPACK_GCC_VER)
XPACK_GCC_REL := https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v$(XPACK_GCC_VER)

# Windows helper utilities (make, etc.) — not the compiler
WINDOWS_TOOLS_UTILS := https://firmware.ardupilot.org/Tools/AM32-tools/windows-tools.zip

ifeq ($(OS),Windows_NT)

# Windows recipes run under cmd.exe (see tools.mk SHELL). Use PowerShell only.
arm_sdk_install:
	@echo Installing windows tools
	@powershell -NoProfile -Command "\
		if (-not (Test-Path 'tools/windows/make/bin/make.exe')) { \
			Write-Host 'downloading windows-tools.zip (make utilities)'; \
			(New-Object System.Net.WebClient).DownloadFile('$(WINDOWS_TOOLS_UTILS)', 'windows-tools.zip'); \
			Write-Host 'unpacking windows-tools.zip'; \
			Expand-Archive -Path windows-tools.zip -Force -DestinationPath .; \
		}; \
		if (-not (Test-Path 'tools/windows/$(XPACK_GCC_DIR)/bin/arm-none-eabi-gcc.exe')) { \
			Write-Host 'downloading $(XPACK_GCC_DIR) (win32-x64)'; \
			(New-Object System.Net.WebClient).DownloadFile( \
				'$(XPACK_GCC_REL)/xpack-arm-none-eabi-gcc-$(XPACK_GCC_VER)-win32-x64.zip', \
				'xpack-gcc-win.zip'); \
			Write-Host 'unpacking xpack gcc into tools/windows/'; \
			New-Item -ItemType Directory -Force -Path tools/windows | Out-Null; \
			Expand-Archive -Path xpack-gcc-win.zip -Force -DestinationPath tools/windows; \
			Remove-Item -Force xpack-gcc-win.zip -ErrorAction SilentlyContinue; \
		} else { \
			Write-Host 'already installed: tools/windows/$(XPACK_GCC_DIR)'; \
		}"
	@echo windows tools install done

else
# MacOS and Linux
UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S),Darwin)

# Prefer native arm64 on Apple Silicon; fall back to x64 (Rosetta).
MAC_XPACK_ARCH := $(shell uname -m | sed 's/x86_64/x64/;s/arm64/arm64/')
ifeq ($(MAC_XPACK_ARCH),arm64)
MAC_XPACK_ASSET := xpack-arm-none-eabi-gcc-$(XPACK_GCC_VER)-darwin-arm64.tar.gz
else
MAC_XPACK_ASSET := xpack-arm-none-eabi-gcc-$(XPACK_GCC_VER)-darwin-x64.tar.gz
endif

arm_sdk_install:
	@echo Installing macos tools \(gcc $(XPACK_GCC_VER)\)
	@if [ ! -x tools/macos/$(XPACK_GCC_DIR)/bin/arm-none-eabi-gcc ]; then \
		echo "downloading $(MAC_XPACK_ASSET)"; \
		mkdir -p tools/macos downloads; \
		wget -q -O downloads/$(MAC_XPACK_ASSET) $(XPACK_GCC_REL)/$(MAC_XPACK_ASSET); \
		tar -xzf downloads/$(MAC_XPACK_ASSET) -C tools/macos; \
	else \
		echo "already installed: tools/macos/$(XPACK_GCC_DIR)"; \
	fi
	@echo macos tools install done

else

# Linux x64 default (GitHub ubuntu-latest). arm64 runners can override via env.
LINUX_XPACK_ARCH ?= $(shell uname -m | sed 's/x86_64/x64/;s/aarch64/arm64/')
ifeq ($(LINUX_XPACK_ARCH),arm64)
LINUX_XPACK_ASSET := xpack-arm-none-eabi-gcc-$(XPACK_GCC_VER)-linux-arm64.tar.gz
else
LINUX_XPACK_ASSET := xpack-arm-none-eabi-gcc-$(XPACK_GCC_VER)-linux-x64.tar.gz
endif

arm_sdk_install:
	@echo Installing linux tools \(gcc $(XPACK_GCC_VER)\)
	@if [ ! -x tools/linux/$(XPACK_GCC_DIR)/bin/arm-none-eabi-gcc ]; then \
		echo "downloading $(LINUX_XPACK_ASSET)"; \
		mkdir -p tools/linux downloads; \
		wget -q -O downloads/$(LINUX_XPACK_ASSET) $(XPACK_GCC_REL)/$(LINUX_XPACK_ASSET); \
		tar -xzf downloads/$(LINUX_XPACK_ASSET) -C tools/linux; \
	else \
		echo "already installed: tools/linux/$(XPACK_GCC_DIR)"; \
	fi
	@echo linux tools install done

endif
endif
