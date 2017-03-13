
ccflags-$(CONFIG_XENO_DRIVERS_I2C_DEBUG) := -DDEBUG -Wall -Werror

obj-$(CONFIG_XENO_DRIVERS_I2C) += xeno_i2c.o

xeno_i2c-y := i2c-master.o

obj-$(CONFIG_XENO_DRIVERS_I2C_BCM2708) += xeno_i2c_bcm2708.o

xeno_i2c_bcm2708-y := i2c-bcm2708.o
