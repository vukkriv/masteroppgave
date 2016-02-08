#ifndef PLAN_GENERATOR_DUBINSPATH
#define PLAN_GENERATOR_DUBINSPATH

#define xx  0
#define yy  1
#define xx1 0
#define xx2 1
#define xxc 2
#define yyc 3

#define WP_OLD_HEIGHT_CTRL 0
#define MIN_DESCEND_HEIGHT 40

bool initiateValues(double &p_net_hor, double &p_R, double p_WP[2][5], double p_circles[2][4], double &p_land_lat, double &p_land_lon, double &p_land_heading);

#endif //PLAN_GENERATOR_DUBINSPATH
