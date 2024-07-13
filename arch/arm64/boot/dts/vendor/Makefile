vendor := $(srctree)/$(src)

ifneq "$(wildcard $(vendor)/qcom)" ""
	subdir-$(CONFIG_MACH_LGE) += lge
	subdir-y += qcom
endif
