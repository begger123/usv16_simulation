# Type of controller to use
uint8    TYPE_TRANSITING=0
uint8    TYPE_STATION_KEEPING=1
uint8    TYPE_DIRECT_CONTROL=2
uint8    type

# Storage of data - will change depending on context
time timeNow
float64[] data
