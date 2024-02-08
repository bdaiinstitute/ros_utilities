
int8 DATA_NOT_SET=0
int8 DATA_NUMBER_SET=1
int8 DATA_TEXT_SET=2
int8 DATA_PAIR_SET=3

# Numeric value.
float32 number
# Text value.
string text
# Pair value.
proto2ros/Any pair  # is proto2ros_tests/Pair (type-erased)
int8 data_choice  # deprecated
int8 which
