test: venc-rc vin-fi
	python3 -B test_ax620e_media_dtb.py
	python3 -B test_ax630c_npu_dtb.py

# Host build of the encoder's rate control against its reference.
venc-rc:
	$(CC) -O2 -Wall -Wextra -Werror -Wno-unused-function \
		-Ivenc_rc -I../kernel/vc8000e -I../kernel/vc8000e/vcenc \
		venc_rc/test_venc_rc.c -lm -o venc_rc/test_venc_rc
	./venc_rc/test_venc_rc

# Host build of the VIN frame-interval estimate behind G_PARM.
vin-fi:
	$(CC) -O2 -Wall -Wextra -Werror -Ivin_fi -I../kernel \
		vin_fi/test_vin_fi.c -lm -o vin_fi/test_vin_fi
	./vin_fi/test_vin_fi

.PHONY: test venc-rc vin-fi
