/* undefine these as not all std libraries provide them) */
#undef min
#undef max

#define max(a, b)	( (a) > (b) ? (a) : (b) )
#define min(a, b)	( (a) < (b) ? (a) : (b) )

