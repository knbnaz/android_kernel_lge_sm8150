ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_ARCH_SM8150) += \
	sm8150-mh2lm_kr_rev-0.0-overlay.dtbo \
	sm8150-mh2lm_kr_rev-1.0-overlay.dtbo \
	sm8150-mh2lm_kr_rev-1.01-overlay.dtbo \
	sm8150-mh2lm_kr_rev-a-overlay.dtbo \
	sm8150-mh2lm_kr_rev-b-overlay.dtbo \
	sm8150-mh2lm_rev-0.0-overlay.dtbo \
	sm8150-mh2lm_rev-1.0-overlay.dtbo \
	sm8150-mh2lm_rev-a-overlay.dtbo \
	sm8150-mh2lm_rev-b-overlay.dtbo

sm8150-mh2lm_kr_rev-0.0-overlay.dtbo-base := ../../../qcom/sm8150.dtb ../../../qcom/sm8150-v2.dtb
sm8150-mh2lm_kr_rev-1.0-overlay.dtbo-base := ../../../qcom/sm8150.dtb ../../../qcom/sm8150-v2.dtb
sm8150-mh2lm_kr_rev-1.01-overlay.dtbo-base := ../../../qcom/sm8150.dtb ../../../qcom/sm8150-v2.dtb
sm8150-mh2lm_kr_rev-a-overlay.dtbo-base := ../../../qcom/sm8150.dtb ../../../qcom/sm8150-v2.dtb
sm8150-mh2lm_kr_rev-b-overlay.dtbo-base := ../../../qcom/sm8150.dtb ../../../qcom/sm8150-v2.dtb
sm8150-mh2lm_rev-0.0-overlay.dtbo-base := ../../../qcom/sm8150.dtb ../../../qcom/sm8150-v2.dtb
sm8150-mh2lm_rev-1.0-overlay.dtbo-base := ../../../qcom/sm8150.dtb ../../../qcom/sm8150-v2.dtb
sm8150-mh2lm_rev-a-overlay.dtbo-base := ../../../qcom/sm8150.dtb ../../../qcom/sm8150-v2.dtb
sm8150-mh2lm_rev-b-overlay.dtbo-base := ../../../qcom/sm8150.dtb ../../../qcom/sm8150-v2.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
