#ifndef batterylogo_h
#define batterylogo_h

#include "s1/battery0.c"
#include "s1/battery1.c"
#include "s1/battery2.c"
#include "s1/battery3.c"
#include "s1/battery4.c"
#include "s1/battery5.c"
#include "s1/battery6.c"
#include "s1/battery7.c"
#include "s1/battery8.c"
#include "s1/battery9.c"
#include "s1/battery10.c"
#include "s1/battery11.c"
#include "s1/battery12.c"
#include "s1/battery13.c"
#include "s1/battery14.c"
#include "s1/battery15.c"
#include "s1/battery16.c"
#include "s1/battery17.c"
#include "s1/battery18.c"
#include "s1/battery19.c"
#include "s1/battery20.c"
#include "s1/battery21.c"
#include "s2/battery0.c"
#include "s2/battery1.c"
#include "s2/battery2.c"
#include "s2/battery3.c"
#include "s2/battery4.c"
#include "s2/battery5.c"
#include "s2/battery6.c"
#include "s2/battery7.c"
#include "s2/battery8.c"
#include "s2/battery9.c"
#include "s2/battery10.c"
#include "s2/battery11.c"
#include "s2/battery12.c"
#include "s2/battery13.c"
#include "s2/battery14.c"
#include "s2/battery15.c"
#include "s2/battery16.c"
#include "s2/battery17.c"
#include "s2/battery18.c"
#include "s2/battery19.c"
#include "s2/battery20.c"
#include "s2/battery21.c"
#include "s3/battery0.c"
#include "s3/battery1.c"
#include "s3/battery2.c"
#include "s3/battery3.c"
#include "s3/battery4.c"
#include "s3/battery5.c"
#include "s3/battery6.c"
#include "s3/battery7.c"
#include "s3/battery8.c"
#include "s3/battery9.c"
#include "s3/battery10.c"
#include "s3/battery11.c"
#include "s3/battery12.c"
#include "s3/battery13.c"
#include "s3/battery14.c"
#include "s3/battery15.c"
#include "s3/battery16.c"
#include "s3/battery17.c"
#include "s3/battery18.c"
#include "s3/battery19.c"
#include "s3/battery20.c"
#include "s3/battery21.c"
#include "s4/battery0.c"
#include "s4/battery1.c"
#include "s4/battery2.c"
#include "s4/battery3.c"
#include "s4/battery4.c"
#include "s4/battery5.c"
#include "s4/battery6.c"
#include "s4/battery7.c"
#include "s4/battery8.c"
#include "s4/battery9.c"
#include "s4/battery10.c"
#include "s4/battery11.c"
#include "s4/battery12.c"
#include "s4/battery13.c"
#include "s4/battery14.c"
#include "s4/battery15.c"
#include "s4/battery16.c"
#include "s4/battery17.c"
#include "s4/battery18.c"
#include "s4/battery19.c"
#include "s4/battery20.c"
#include "s4/battery21.c"
#include "s5/battery0.c"
#include "s5/battery1.c"
#include "s5/battery2.c"
#include "s5/battery3.c"
#include "s5/battery4.c"
#include "s5/battery5.c"
#include "s5/battery6.c"
#include "s5/battery7.c"
#include "s5/battery8.c"
#include "s5/battery9.c"
#include "s5/battery10.c"
#include "s5/battery11.c"
#include "s5/battery12.c"
#include "s5/battery13.c"
#include "s5/battery14.c"
#include "s5/battery15.c"
#include "s5/battery16.c"
#include "s5/battery17.c"
#include "s5/battery18.c"
#include "s5/battery19.c"
#include "s5/battery20.c"
#include "s5/battery21.c"

#define MAX_LOGO_NUM	22
#define MAX_BAT_LEVEL	5

typedef struct _batterylogo_t
{
    unsigned short width;
    unsigned short height;
    int stride;
    unsigned char bpp;
    int byte[MAX_BAT_LEVEL][MAX_LOGO_NUM];
    unsigned short *data[MAX_BAT_LEVEL][MAX_LOGO_NUM];
}batterylogo_t;

static const
batterylogo_t s_batterylogo =
{
    160, //width
    260, //height
    640, //stride
    24,  //bpp

    {//byte
	{//battery level 0
	66264, 66192, 88140, 86622, 87036, 92592, 91476, 96540, 99642, 103152,
	105648, 105972, 111300, 114450, 116658, 123960, 129666, 129768, 129378, 129378,
	129378, 129378,
	},
    {//battery level 1
	68388, 67968, 88050, 86814, 87084, 93210, 91704, 95880, 98082, 101580,
	103746, 104040, 109398, 111942, 115314, 120384, 126120, 125442, 125238, 125238,
	125238, 125238,
	},
	{//battery level 2
	72570, 72348, 89736, 89754, 90060, 93114, 93978, 98802, 100152, 103482,
	105222, 105984, 110982, 113958, 115290, 120186, 125658, 125280, 125760, 125760,
	125760, 125760,
    },
	{//battery level 3
	76920, 77064, 91488, 92448, 93420, 95076, 97056, 100164, 102072, 101466,
	105858, 107484, 111882, 113232, 115194, 119310, 124284, 123222, 124416, 105858, 
	124416, 124416,
	},
    {//battery level 4
	75030, 74940, 87456, 90222, 89688, 91158, 92100, 92820, 97812, 96792,
	101292, 103512, 105606, 108492, 109812, 112254, 116772, 115536, 116964, 116964,
	116964, 116964,
	},
    },

    {//data
	{//battery level 0
	&s1_batterylogo0[0],
	&s1_batterylogo1[0],
	&s1_batterylogo2[0],
	&s1_batterylogo3[0],
	&s1_batterylogo4[0],
	&s1_batterylogo5[0],
	&s1_batterylogo6[0],
	&s1_batterylogo7[0],
	&s1_batterylogo8[0],
	&s1_batterylogo9[0],
	&s1_batterylogo10[0],
	&s1_batterylogo11[0],
	&s1_batterylogo12[0],
	&s1_batterylogo13[0],
	&s1_batterylogo14[0],
	&s1_batterylogo15[0],
	&s1_batterylogo16[0],
	&s1_batterylogo17[0],
	&s1_batterylogo18[0],
	&s1_batterylogo19[0],
	&s1_batterylogo20[0],
	&s1_batterylogo21[0],
	},
	{//battery level 1
	&s2_batterylogo0[0],
	&s2_batterylogo1[0],
	&s2_batterylogo2[0],
	&s2_batterylogo3[0],
	&s2_batterylogo4[0],
	&s2_batterylogo5[0],
	&s2_batterylogo6[0],
	&s2_batterylogo7[0],
	&s2_batterylogo8[0],
	&s2_batterylogo9[0],
	&s2_batterylogo10[0],
	&s2_batterylogo11[0],
	&s2_batterylogo12[0],
	&s2_batterylogo13[0],
	&s2_batterylogo14[0],
	&s2_batterylogo15[0],
	&s2_batterylogo16[0],
	&s2_batterylogo17[0],
	&s2_batterylogo18[0],
	&s2_batterylogo19[0],
	&s2_batterylogo20[0],
	&s2_batterylogo21[0],
	},
	{//battery level 2
	&s3_batterylogo0[0],
	&s3_batterylogo1[0],
	&s3_batterylogo2[0],
	&s3_batterylogo3[0],
	&s3_batterylogo4[0],
	&s3_batterylogo5[0],
	&s3_batterylogo6[0],
	&s3_batterylogo7[0],
	&s3_batterylogo8[0],
	&s3_batterylogo9[0],
	&s3_batterylogo10[0],
	&s3_batterylogo11[0],
	&s3_batterylogo12[0],
	&s3_batterylogo13[0],
	&s3_batterylogo14[0],
	&s3_batterylogo15[0],
	&s3_batterylogo16[0],
	&s3_batterylogo17[0],
	&s3_batterylogo18[0],
	&s3_batterylogo19[0],
	&s3_batterylogo20[0],
	&s3_batterylogo21[0],
	},
	{//battery level 3
	&s4_batterylogo0[0],
	&s4_batterylogo1[0],
	&s4_batterylogo2[0],
	&s4_batterylogo3[0],
	&s4_batterylogo4[0],
	&s4_batterylogo5[0],
	&s4_batterylogo6[0],
	&s4_batterylogo7[0],
	&s4_batterylogo8[0],
	&s4_batterylogo9[0],
	&s4_batterylogo10[0],
	&s4_batterylogo11[0],
	&s4_batterylogo12[0],
	&s4_batterylogo13[0],
	&s4_batterylogo14[0],
	&s4_batterylogo15[0],
	&s4_batterylogo16[0],
	&s4_batterylogo17[0],
	&s4_batterylogo18[0],
	&s4_batterylogo19[0],
	&s4_batterylogo20[0],
	&s4_batterylogo21[0],
	},
	{//battery level 4
	&s5_batterylogo0[0],
	&s5_batterylogo1[0],
	&s5_batterylogo2[0],
	&s5_batterylogo3[0],
	&s5_batterylogo4[0],
	&s5_batterylogo5[0],
	&s5_batterylogo6[0],
	&s5_batterylogo7[0],
	&s5_batterylogo8[0],
	&s5_batterylogo9[0],
	&s5_batterylogo10[0],
	&s5_batterylogo11[0],
	&s5_batterylogo12[0],
	&s5_batterylogo13[0],
	&s5_batterylogo14[0],
	&s5_batterylogo15[0],
	&s5_batterylogo16[0],
	&s5_batterylogo17[0],
	&s5_batterylogo18[0],
	&s5_batterylogo19[0],
	&s5_batterylogo20[0],
	&s5_batterylogo21[0],
	},
    },
};

const
batterylogo_t *batterylogo = &s_batterylogo;
#endif

