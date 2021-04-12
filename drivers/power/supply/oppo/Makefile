ifeq ($(CONFIG_OPPO_CHARGER_MTK),y)
subdir-ccflags-y += -Werror
subdir-ccflags-y += -Wno-unused-variable
endif

obj-y      += charger_ic/
obj-y      += gauge_ic/
obj-y      += vooc_ic/
obj-y      += adapter_ic/

obj-y	+= oppo_charger.o
obj-y	+= oppo_gauge.o
obj-y	+= oppo_vooc.o
obj-y	+= oppo_adapter.o
obj-y	+= oppo_short.o
#ifdef ODM_HQ_EDIT
#zhangchao@ODM.HQ.Charger 2019/12/06 modified for USB_NTC
obj-y	+= oppo_usb_ntc1.o
obj-y	+= oppo_usb_ntc2.o
obj-y	+= oppo_flashlight_ntc.o
obj-y	+= oppo_charger_ntc.o
#endif