# Registers and values required


### Notes on sensor setup
- Disable I2C in CTRL4_C
- Set Accelerometer operation mode in register CTRL1_XL (0x10)
- Set Gyroscope operation mode in register CTRL2_G (0x11)
- Software Reset, Register Increment, and interrupt activation level set in CTRL3_C (0x12)
- Disable I3C using CTRL9_XL



### Notes on Interrrupt setup
- Interrupt Setup is configured by switching to Embedded Function control mode by writing 0x1 to the FUNC_CFG_ACCESS (0x1) register
- Enable "Significant Motion Detection" in register EMB_FUNC_EN_A (04h)
- Route SMD interrupt to INT1 and Inactivity to INT2

- Interrupt status cleared by reading interrupt register
    - For SMD interrupt, read EMB_FUNC_STATUS_MAINPAGE (0x35) register
- TAP has inactivity sensor. Activity interrupt can be used to wake processor from sleep


### Notes on Data Acquisition
- Data is back into two bytes, stored in Two's Complement format
- L register is bits 7-0, H register is 15-8
- Accelerometer data registers:
    - OUTX_L_A (0x28) and OUTX_H_A (0x29) - X axis linear acceleration
    - OUTY_L_A (0x2A) and OUTY_H_A (0x2B) - Y axis linear acceleration
    - OUTZ_L_A (0x2C) and OUTZ_H_A (0x2D) - Z axis linear acceleration
- Gyroscope data registers:
    - OUTX_L_G (0x22) and OUTX_H_G (0x23) - X axis angular rate value
    - OUTY_L_G (0x24) and OUTY_H_G (0x25) - Y axis angular rate value
    - OUTZ_L_G (0x26) and OUTZ_H_G (0x27) - Z axis angular rate value
- Temperature data registers:
    - OUT_TEMP_LOW (0x20) and OUT_TEMP_HIGH (0x21) - Temperature output register


# Cheatsheet

# A first-level heading
## A second-level heading
### A third-level heading


Style	Syntax	Keyboard shortcut	Example	Output
Bold	** ** or __ __	Command+B (Mac) or Ctrl+B (Windows/Linux)	**This is bold text**	This is bold text
Italic	* * or _ _     	Command+I (Mac) or Ctrl+I (Windows/Linux)	_This text is italicized_	This text is italicized
Strikethrough	~~ ~~	None	~~This was mistaken text~~	This was mistaken text
Bold and nested italic	** ** and _ _	None	**This text is _extremely_ important**	This text is extremely important
All bold and italic	*** ***	None	***All this text is important***	All this text is important
Subscript	<sub> </sub>	None	This is a <sub>subscript</sub> text	This is a subscript text
Superscript	<sup> </sup>	None	This is a <sup>superscript</sup> text	This is a superscript text
Underline	<ins> </ins>	None	This is an <ins>underlined</ins> text	This text is underlined