test: venc-rc
	python3 -B test_ax620e_media_dtb.py
	python3 -B test_ax630c_npu_dtb.py

# Host build of the encoder's rate control against its reference.
venc-rc:
	$(CC) -O2 -Wall -Wextra -Werror -Wno-unused-function \
		-Ivenc_rc -I../kernel/vc8000e -I../kernel/vc8000e/vcenc \
		venc_rc/test_venc_rc.c -lm -o venc_rc/test_venc_rc
	./venc_rc/test_venc_rc

.PHONY: test venc-rc
