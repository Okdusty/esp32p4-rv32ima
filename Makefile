.PHONY: guest guest-rootfs guest-image guest-dtb guest-verify test \
	test-emulator clean distclean

guest:
	./scripts/build-guest.sh

guest-rootfs:
	./scripts/build-openwrt-rootfs.sh

guest-image:
	./scripts/build-linux-image.sh

guest-dtb:
	./scripts/build-dtb.sh all

guest-verify:
	./scripts/verify-guest-image.py main/Image

test: test-emulator

test-emulator:
	./scripts/test-emulator.sh

clean:
	./scripts/clean-guest.sh clean

distclean:
	./scripts/clean-guest.sh distclean
