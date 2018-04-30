dtb-$(CONFIG_ARCH_MSM8998) += \
	mata-evt3.dtb \
	mata-evt4.dtb \
	mata-evt4-f.dtb \
	mata-dvt.dtb \
	mata-dvt-f.dtb \
	mata-dvt-g.dtb

ifeq ($(CONFIG_ARM64),y)
always          := $(dtb-y)
subdir-y        := $(dts-dirs)
else
targets += dtbs
targets += $(addprefix ../, $(dtb-y))

$(obj)/../%.dtb: $(src)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(dtb-y))
endif
clean-files := *.dtb
