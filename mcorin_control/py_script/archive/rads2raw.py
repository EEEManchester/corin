def rads2raw(rads):
	## convert joint velocity to dxl raw value
	velocity_to_value_ratio_ = 1.0/(0.229*2*3.14159265/60.0)

	value = int(round(abs(rads) * velocity_to_value_ratio_))
	# force velocity to move at slowest
	if (value == 0):
		value = 1
	return value # int(math.ceil(abs(rads) * velocity_to_value_ratio_))