.PHONY: guest guest-rootfs guest-image guest-dtb guest-verify clean distclean

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

clean:
	./scripts/clean-guest.sh clean

distclean:
	./scripts/clean-guest.sh distclean
