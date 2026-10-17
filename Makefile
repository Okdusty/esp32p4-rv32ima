.PHONY: guest guest-rootfs guest-image guest-dtb guest-desktop guest-verify test \
	test-emulator test-reboot test-palette clean distclean

guest:
	./scripts/build-guest.sh

guest-rootfs:
	./scripts/build-openwrt-rootfs.sh

guest-image:
	./scripts/build-linux-image.sh

guest-dtb:
	./scripts/build-dtb.sh all

guest-desktop:
	./scripts/build-desktop-rofs.sh

guest-verify:
	./scripts/verify-guest-image.py main/Image

test: test-emulator test-reboot test-palette

test-emulator:
	./scripts/test-emulator.sh

test-reboot:
	./scripts/test-reboot.sh

test-palette:
	./scripts/test-rgb565.sh

clean:
	./scripts/clean-guest.sh clean

distclean:
	./scripts/clean-guest.sh distclean
