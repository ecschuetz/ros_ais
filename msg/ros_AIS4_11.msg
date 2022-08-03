#AIS BASE STATION REPORT (MESSAGE 4) AND COORDINATED UNIVERSAL TIME AND DATE RESPONSE (MESSAGE 11)

Header header

int32 id   #Identifier for this message 1, 2 or 3

int32 repeat_indicator   #Used by the repeater to indicate how many times a message has been repeated. See Section 4.6.1, Annex 2; 0-3; 0 = default; 3 = do not repeat any more.

int32 mmsi   #MMSI number

int32 year

int32 month

int32 day

int32 hour

int32 minute

int32 second

int32 position_accuracy

float64 latitude

float64 longitude

int32 fix_type

int32 transmission_ctl

float64 spare

int32 raim
