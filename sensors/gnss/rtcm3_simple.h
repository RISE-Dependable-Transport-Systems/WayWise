/*
 *     Copyright 2017 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#ifndef RTCM3_SIMPLE_H
#define RTCM3_SIMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>


#ifndef D
#define D(x) 						((double)x##L)
#endif

#ifndef D_PI
#define D_PI						D(3.14159265358979323846)
#endif

// Defines
#define RTCM3PREAMB		0xD3                // rtcm ver.3 frame preamble
#define CODE_L1C        1                   // obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS)
#define CODE_L1P        2                   // obs code: L1P,G1P    (GPS,GLO)
#define CODE_L2C        14                  // obs code: L2C/A,G1C/A (GPS,GLO)
#define CODE_L2P        19                  // obs code: L2P,G2P    (GPS,GLO)
#define CODE_L2W        20                  // obs code: L2 Z-track (GPS)

#define SYS_NONE        0x00                // navigation system: none
#define SYS_GPS         0x01                // navigation system: GPS
#define SYS_SBS         0x02                // navigation system: SBAS
#define SYS_GLO         0x04                // navigation system: GLONASS
#define SYS_GAL         0x08                // navigation system: Galileo
#define SYS_QZS         0x10                // navigation system: QZSS
#define SYS_CMP         0x20                // navigation system: BeiDou
#define SYS_LEO         0x40                // navigation system: LEO
#define SYS_ALL         0xFF                // navigation system: all

// Datatypes
typedef struct {
    double t_tow;       // Time of week (GPS)
    double t_tod;       // Time of day (GLONASS)
    double t_wn;        // Week number
    int staid;          // ref station id
    bool sync;          // True if more messages are coming
    int type;           // RTCM Type
} rtcm_obs_header_t;

typedef struct {
    double P[2];        // Pseudorange observation
    double L[2];        // Carrier phase observation
    uint8_t cn0[2];     // Carrier-to-Noise density [dB Hz]
    uint8_t lock[2];    // Lock. Set to 0 when the lock has changed, 127 otherwise. TODO: is this correct?
    uint8_t prn;        // Sattelite
    uint8_t freq;       // Frequency slot (GLONASS)
    uint8_t code[2];    // Code indicator
} rtcm_obs_t;

typedef struct {
    int staid;
    double lat;
    double lon;
    double height;
    double ant_height;
} rtcm_ref_sta_pos_t;

typedef struct {
    double tgd;           // Group delay differential between L1 and L2 [s]
    double c_rs;          // Amplitude of the sine harmonic correction term to the orbit radius [m]
    double c_rc;          // Amplitude of the cosine harmonic correction term to the orbit radius [m]
    double c_uc;          // Amplitude of the cosine harmonic correction term to the argument of latitude [rad]
    double c_us;          // Amplitude of the sine harmonic correction term to the argument of latitude [rad]
    double c_ic;          // Amplitude of the cosine harmonic correction term to the angle of inclination [rad]
    double c_is;          // Amplitude of the sine harmonic correction term to the angle of inclination [rad]
    double dn;            // Mean motion difference [rad/s]
    double m0;            // Mean anomaly at reference time [radians]
    double ecc;           // Eccentricity of satellite orbit
    double sqrta;         // Square root of the semi-major axis of orbit [m^(1/2)]
    double omega0;        // Longitude of ascending node of orbit plane at weekly epoch [rad]
    double omegadot;      // Rate of right ascension [rad/s]
    double w;             // Argument of perigee [rad]
    double inc;           // Inclination [rad]
    double inc_dot;       // Inclination first derivative [rad/s]
    double af0;           // Polynomial clock correction coefficient (clock bias) [s]
    double af1;           // Polynomial clock correction coefficient (clock drift) [s/s]
    double af2;           // Polynomial clock correction coefficient (rate of clock drift) [s/s^2]
    double toe_tow;       // Time of week [s]
    uint16_t toe_wn;      // Week number [week]
    double toc_tow;       // Clock reference time of week [s]
    int sva;              // SV accuracy (URA index)
    int svh;              // SV health (0:ok)
    int code;             // GPS/QZS: code on L2, GAL/CMP: data sources
    int flag;             // GPS/QZS: L2 P data flag, CMP: nav type
    double fit;           // fit interval (h)
    uint8_t prn;          // Sattelite
    uint8_t iode;         // Issue of ephemeris data
    uint16_t iodc;        // Issue of clock data
} rtcm_ephemeris_t;

typedef struct {
    bool decode_all;
    int buffer_ptr;
    int len;
    uint8_t buffer[1100];
    rtcm_obs_header_t header;
    rtcm_obs_t obs[64];
    rtcm_ref_sta_pos_t pos;
    rtcm_ephemeris_t eph;
    void(*rx_rtcm_obs)(rtcm_obs_header_t *header, rtcm_obs_t *obs, int obs_num);
    void(*rx_rtcm_1005_1006)(rtcm_ref_sta_pos_t *pos);
    void(*rx_rtcm_1019)(rtcm_ephemeris_t *eph);
    void(*rx_rtcm)(uint8_t *data, int len, int type);
} rtcm3_state;

// Functions
void rtcm3_set_rx_callback_obs(void(*func)(rtcm_obs_header_t *header, rtcm_obs_t *obs, int obs_num), rtcm3_state *state);
void rtcm3_set_rx_callback_1005_1006(void(*func)(rtcm_ref_sta_pos_t *pos), rtcm3_state *state);
void rtcm3_set_rx_callback_1019(void(*func)(rtcm_ephemeris_t *eph), rtcm3_state *state);
void rtcm3_set_rx_callback(void(*func)(uint8_t *data, int len, int type), rtcm3_state *state);
void rtcm3_init_state(rtcm3_state *state);
int rtcm3_input_data(uint8_t data, rtcm3_state *state);
int rtcm3_encode_1002(rtcm_obs_header_t *header, rtcm_obs_t *obs,
                      int obs_num, uint8_t *buffer, int *buffer_len);
int rtcm3_encode_1010(rtcm_obs_header_t *header, rtcm_obs_t *obs,
                      int obs_num, uint8_t *buffer, int *buffer_len);
int rtcm3_encode_1006(rtcm_ref_sta_pos_t pos, uint8_t *buffer, int *buffer_len);
int rtcm3_encode_1019(rtcm_ephemeris_t *eph, uint8_t *buffer, int *buffer_len);

#ifdef __cplusplus
}
#endif

#endif // RTCM3_SIMPLE_H

