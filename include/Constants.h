#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace ORB_SLAM3 {

    class Constants {

        public:
        enum ePolcamMode {
            POLCAM0 = 0,
            POLCAM01 = 1,
            POLCAM02 = 2,
            POLCAM03 = 3,
            POLCAM12 = 4,
            POLCAM13 = 5,
            POLCAM23 = 6,
            POLCAM012 = 7,
            POLCAM013 = 8,
            POLCAM123 = 9,
            POLCAM1234 = 10,
        };
    };

} // namespace ORB_SLAM3

#endif // CONSTANTS_H
