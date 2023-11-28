#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <vector>
#include <string>
#include "rtl-sdr.h"

#define MODES_DEFAULT_RATE         2500000      /* Some RTL-SDR radios output errors with this sample rate but it is required to properly detect the SSR interrogations */
#define MODES_DEFAULT_FREQ         1030000000   /* Ssr interrogation uplink frequency */
#define MODES_ASYNC_BUF_NUMBER     0            /* Default value 15 */
#define MODES_DATA_LEN             262144       /* Default value 32*16*512 = 262 144 for rtl sdr buffer size if set to 0*/
#define MODES_AUTO_GAIN            -100         /* Use automatic gain. */
#define MODES_MAX_GAIN             999999
#define NOICE_RATIO                0.25         /* Default value 0.3 */
#define NOICE_RATIO_CLOSE          0.75         /* Default value 0.9 */
#define AMP_DIFFERENCE             10           /* Default value */
#define AMP_DIFFERENCE_CLOSE       5           /* Default value */

using namespace std;

struct {
    pthread_t reader_thread;
    pthread_mutex_t data_mutex;     /* Mutex to synchronize buffer access. */
    pthread_cond_t data_cond;       /* Conditional variable associated. */

    /* Data processing related variables */
    unsigned char *data;
    int type;
    uint8_t *magnitude;
    uint8_t *maglut;
    uint32_t data_length;
    bool data_ready;

    /* User definable variables */
    float diffratio;
    float diffratioclose;
    float diffratiop4;
    float diffratioclosep4;
    uint8_t diff;
    uint8_t diffclose;
    bool print_order;
    uint8_t max_noicefloor;
    uint8_t min_peak_amp;
    uint8_t max_noicefloor_close;
    bool baselinemode; /* Calculates averages of detected messages based on value type and outputs them for later use as a baseline values */
    bool print_detected;
    bool downlink; /* True means that it is scanning for uplink signals while false is scanning for downlink replys. NOT IN USE */
    int samplerate;
    bool print_all;
    bool continuous;

    /* Statistics/Results */
    int cumulative_countm;
    int cumulative_count_a;
    int cumulative_count_c;
    int cumulative_count_a_acac;
    int cumulative_count_c_acac;
    int cumulative_count_a_acsac;
    int cumulative_count_c_acsac;
    int cumulative_count_s;
    int countm;
    int count_a;
    int count_c;
    int count_a_acac;
    int count_c_acac;
    int count_a_acsac;
    int count_c_acsac;
    int count_s;
    unsigned char *order;


    /* Test file handling */
    int fd;
    char *filename;


    /* RTL-SDR */
    int freq;
    rtlsdr_dev_t *dev;
    int dev_index;
    int gain;
    int enable_agc;
} Modes;

/* Initialization */
void modesInit(void) {
    Modes.data_length = MODES_DATA_LEN;
    Modes.gain = MODES_MAX_GAIN;
    Modes.dev_index = 0;
    Modes.freq = MODES_DEFAULT_FREQ;
    Modes.samplerate = MODES_DEFAULT_RATE;
    Modes.filename = NULL;
    Modes.cumulative_countm = 0;
    Modes.cumulative_count_a = 0;
    Modes.cumulative_count_c = 0;
    Modes.cumulative_count_a_acac = 0;
    Modes.cumulative_count_c_acac = 0;
    Modes.cumulative_count_a_acsac = 0;
    Modes.cumulative_count_c_acsac = 0;
    Modes.cumulative_count_s = 0;
    Modes.countm = 0;
    Modes.count_a = 0;
    Modes.count_c = 0;
    Modes.count_a_acac = 0;
    Modes.count_c_acac = 0;
    Modes.count_a_acsac = 0;
    Modes.count_c_acsac = 0;
    Modes.count_s = 0;
    Modes.enable_agc = 0;
    Modes.diff = AMP_DIFFERENCE;
    Modes.diffclose = AMP_DIFFERENCE_CLOSE;
    Modes.diffratiop4 = NOICE_RATIO + 0.25;
    Modes.diffratioclosep4 = NOICE_RATIO_CLOSE + 0.75;
    Modes.diffratioclose = NOICE_RATIO_CLOSE;
    Modes.diffratio = NOICE_RATIO;
    Modes.max_noicefloor_close = 255;
    Modes.print_order = false;
    Modes.min_peak_amp = 0;
    Modes.max_noicefloor = 255;
    Modes.baselinemode = false;
    Modes.print_detected = false;
    Modes.print_all = false;
    Modes.continuous = false;
    pthread_mutex_init(&Modes.data_mutex,NULL);
    pthread_cond_init(&Modes.data_cond,NULL);
    Modes.data_ready = false;
}

void dataInit(void) {
    if (Modes.filename != NULL)
    {
        off_t size;
        Modes.fd = open(Modes.filename, O_RDONLY);
        size = lseek(Modes.fd, 0, SEEK_END);
        Modes.data_length = size;
    }

    if ((Modes.order = (unsigned char*) malloc(Modes.data_length)) == NULL)
    {
        printf("Out of memory allocating data buffer.\n");
        exit(1);
    }

    Modes.order[0] = 0;

    if ((Modes.data = (unsigned char*) malloc(Modes.data_length/2)) == NULL ||
    (Modes.magnitude = (uint8_t *) malloc(Modes.data_length)) == NULL)
    {
        printf("Out of memory allocating data buffer.\n");
        exit(1);
    }
}

/* RTL-SDR initialization */
void modesInitRTLSDR(void) {
    int j;
    int device_count;
    int ppm_error = 0;
    char vendor[256], product[256], serial[256];

    device_count = rtlsdr_get_device_count();
    if (!device_count) {
        fprintf(stderr, "No supported RTLSDR devices found.\n");
        exit(1);
    }

    fprintf(stderr, "Found %d device(s):\n", device_count);
    for (j = 0; j < device_count; j++) {
        rtlsdr_get_device_usb_strings(j, vendor, product, serial);
        fprintf(stderr, "%d: %s, %s, SN: %s %s\n", j, vendor, product, serial,
            (j == Modes.dev_index) ? "(currently selected)" : "");
    }

    if (rtlsdr_open(&Modes.dev, Modes.dev_index) < 0) {
        fprintf(stderr, "Error opening the RTLSDR device: %s\n",
            strerror(errno));
        exit(1);
    }

    /* Set gain, frequency, sample rate, and reset the device. */
    rtlsdr_set_tuner_gain_mode(Modes.dev,
        (Modes.gain == MODES_AUTO_GAIN) ? 0 : 1);
    if (Modes.gain != MODES_AUTO_GAIN) {
        if (Modes.gain == MODES_MAX_GAIN) {
            /* Find the maximum gain available. */
            int numgains;
            int gains[100];

            numgains = rtlsdr_get_tuner_gains(Modes.dev, gains);
            Modes.gain = gains[numgains-1];
            fprintf(stderr, "Max available gain is: %.2f\n", Modes.gain/10.0);
        }
        rtlsdr_set_tuner_gain(Modes.dev, Modes.gain);
        fprintf(stderr, "Setting gain to: %.2f\n", Modes.gain/10.0);
    } else {
        fprintf(stderr, "Using automatic gain control.\n");
    }
    rtlsdr_set_freq_correction(Modes.dev, ppm_error);
    if (Modes.enable_agc) rtlsdr_set_agc_mode(Modes.dev, 1);
    rtlsdr_set_center_freq(Modes.dev, Modes.freq);
    rtlsdr_set_sample_rate(Modes.dev, MODES_DEFAULT_RATE);
    rtlsdr_reset_buffer(Modes.dev);
    fprintf(stderr, "Gain reported by device: %.2f\n",
        rtlsdr_get_tuner_gain(Modes.dev)/10.0);
}



/* Detects and counts different mode a, c and s messages from magnitude vector data.
* Checks first simpler and smaller patterns before moving to longer checks.
* Baseline mode calculates averages of accepted messages and outputs baseline values
* that can be used later on. */
void detectMode(uint8_t *m) {
    int i;
    int a;
    int c;
    int o = 0;
    int os; /* offset that depends on if it is Mode A or C message.  */

    /* Sets zeroes to ones in amplitude data to avoid floation point exceptions. */
    for(i = 0; i<Modes.data_length/2; i++) {
        if (m[i] == 0) { m[i] = 1; }
    }

    if (Modes.freq == 1030000000 && Modes.samplerate == 2500000)
    {
        vector<uint8_t> averagenf;
        vector<uint8_t> averagepulse;
        vector<uint8_t> averagenfclose;
        for (i = 0; i<Modes.data_length/2; i++) {

            if (Modes.print_all == true)
            {
                printf(" %d", m[i]);
            }

            /* Checks existence of P1 pulse and non pulse values that exist in all Mode A/C/S messages */
            if  ((float) m[i+2]/m[i] > Modes.diffratioclose || (float) m[i+2]/m[i+1] > Modes.diffratioclose ||
                (float) m[i+3]/m[i] > Modes.diffratio || (float) m[i+3]/m[i+1] > Modes.diffratio ||
                 m[i]<=m[i+2]+Modes.diff || m[i+1]<=m[i+2]+Modes.diff ||
                 m[i]<=m[i+3]+Modes.diff || m[i+1]<=m[i+3]+Modes.diff ||
                 m[i]<=m[i+4]+Modes.diff || m[i+1]<=m[i+4]+Modes.diff ||
                 m[i]<=m[i+7]+Modes.diff || m[i+1]<=m[i+7]+Modes.diff ||
                 m[i]<=Modes.min_peak_amp || m[i+1]<=Modes.min_peak_amp ||
                 m[i+2]>=Modes.max_noicefloor_close || m[i+3]>=Modes.max_noicefloor)
            {
                goto next_loop;
            }

            /* Check existence of valid Mode S preample. If there is P3 pulse 2 microseconds
            * after start the message is Mode S message. */
            if (m[i+5] >= m[i+3]+Modes.diff &&
                m[i+6] >= m[i+3]+Modes.diff &&
                m[i+5] >= m[i+4]+Modes.diffclose &&
                m[i+6] >= m[i+4]+Modes.diffclose &&
                m[i+5] >= m[i+2]+Modes.diffclose &&
                m[i+6] >= m[i+2]+Modes.diffclose &&
                m[i+5] >= m[i+7]+Modes.diffclose &&
                m[i+6] >= m[i+7]+Modes.diffclose &&
                m[i+5] >= m[i+8]+Modes.diffclose &&
                m[i+6] >= m[i+8]+Modes.diffclose &&
                m[i+5] >= Modes.min_peak_amp &&
                m[i+6] >= Modes.min_peak_amp &&
                m[i+2] <= Modes.max_noicefloor_close &&
                m[i+3] <= Modes.max_noicefloor &&
                m[i+4] <= Modes.max_noicefloor_close &&
                m[i+7] <= Modes.max_noicefloor_close &&
                m[i+8] <= Modes.max_noicefloor_close &&
                (float) m[i+4]/m[i] < Modes.diffratioclose &&
                (float) m[i+4]/m[i+1] < Modes.diffratioclose &&
                (float) m[i+7]/m[i] < Modes.diffratioclose &&
                (float) m[i+7]/m[i+1] < Modes.diffratioclose &&
                (float) m[i+3]/m[i+5] < Modes.diffratio &&
                (float) m[i+3]/m[i+6] < Modes.diffratio &&
                (float) m[i+4]/m[i+5] < Modes.diffratioclose &&
                (float) m[i+4]/m[i+6] < Modes.diffratioclose &&
                (float) m[i+2]/m[i+5] < Modes.diffratioclose &&
                (float) m[i+2]/m[i+6] < Modes.diffratioclose &&
                (float) m[i+7]/m[i+5] < Modes.diffratioclose &&
                (float) m[i+7]/m[i+6] < Modes.diffratioclose &&
                (float) m[i+8]/m[i+5] < Modes.diffratioclose &&
                (float) m[i+8]/m[i+6] < Modes.diffratioclose )
            {
                Modes.type = 3;
                Modes.countm++;
                Modes.count_s++;
                Modes.order[o] = 3;
                o++;
                if (Modes.baselinemode == true)
                {
                    averagenfclose.push_back(m[i+4]);
                    averagenfclose.push_back(m[i+2]);
                    averagenfclose.push_back(m[i+7]);
                    averagenf.push_back(m[i+3]);
                    averagepulse.push_back(m[i+5]);
                    averagepulse.push_back(m[i+6]);
                    averagepulse.push_back(m[i+1]);
                    averagepulse.push_back(m[i]);
                }
                if (Modes.print_detected == true)
                {
                    printf("Mode S message in starting from bit number: %d ", i);
                    printf(" %d" " %d" " %d" " %d" " %d" " %d" " %d" " %d" " %d\n\n", m[i], m[i+1], m[i+2], m[i+3], m[i+4], m[i+5], m[i+6], m[i+7], m[i+8]);
                }
                i += 49;
                goto next_loop;
            }
            /* Checks if Mode A message. Mode A message has 7,2 microseconds
            * between end of P1 and start of P3. */
            if (m[i+20]<Modes.min_peak_amp || m[i+21]<Modes.min_peak_amp) { goto mode_c_check; }
            for (a = 3; a < 19; a++)
            {
                if (m[i+a]+Modes.diff>=m[i+20] || m[i+a]/(float) m[i+20] > Modes.diffratio) { goto mode_c_check; }
                if (m[i+a]+Modes.diff>=m[i+21] || m[i+a]/(float) m[i+21] > Modes.diffratio) { goto mode_c_check; }
                if (m[i+a]>Modes.max_noicefloor) { goto mode_c_check; }
            }
            if (m[i+22]+Modes.diffclose>=m[i+20]   || m[i+22]+Modes.diffclose>=m[i+21] ||
                m[i+23]+Modes.diff>=m[i+20]        || m[i+23]+Modes.diff>=m[i+21]      ||
                m[i+24]+Modes.diffclose>=m[i+20]   || m[i+24]+Modes.diffclose>=m[i+21] ||
                m[i+19]+Modes.diffclose>=m[i+20]   || m[i+19]+Modes.diffclose>=m[i+21] ||
                m[i+2]+Modes.diffclose>=m[i+20]    || m[i+2]+Modes.diffclose>=m[i+21]  ||
                m[i+22]>Modes.max_noicefloor_close ||
                m[i+23]>Modes.max_noicefloor       ||
                m[i+24]>Modes.max_noicefloor       ||
                m[i+19]>Modes.max_noicefloor_close ||
                m[i+20]<Modes.min_peak_amp         ||
                m[i+21]<Modes.min_peak_amp         ||
                (float) m[i+2]/m[i+20] > Modes.diffratioclose  ||
                (float) m[i+2]/m[i+21] > Modes.diffratioclose  ||
                (float) m[i+22]/m[i+20] > Modes.diffratioclose ||
                (float) m[i+22]/m[i+21] > Modes.diffratioclose ||
                (float) m[i+23]/m[i+20] > Modes.diffratio      ||
                (float) m[i+23]/m[i+21] > Modes.diffratio) { goto mode_c_check; }
            Modes.type = 1;
            os = 25;
            goto p4_check;

        mode_c_check:
            /* Checks the message is Mode C message. Mode C message has 20,2 microseconds
            * between end of P1 and P3. */
            for (c = 3; c < 51; c++) {
                if (m[i+c]+Modes.diff>m[i+52] || m[i+c]/m[i+52] > Modes.diffratio) { goto next_loop;}
                if (m[i+c]+Modes.diff>m[i+53] || m[i+c]/m[i+53] > Modes.diffratio) { goto next_loop;}
                if (m[i+c]>Modes.max_noicefloor) { goto next_loop; }
            }
            if (m[i+54]+Modes.diffclose>=m[i+52] || m[i+54]+Modes.diffclose>=m[i+53] ||
                m[i+55]+Modes.diff>=m[i+52]      || m[i+55]+Modes.diff>=m[i+53]      ||
                m[i+56]+Modes.diffclose>=m[i+52] || m[i+56]+Modes.diffclose>=m[i+53] ||
                m[i+2]+Modes.diffclose>=m[i+52]  || m[i+51]+Modes.diffclose>=m[i+53] ||
                m[i+54]>Modes.max_noicefloor_close ||
                m[i+55]>Modes.max_noicefloor       ||
                m[i+56]>Modes.max_noicefloor       ||
                m[i+51]>Modes.max_noicefloor_close ||
                m[i+52]<Modes.min_peak_amp         ||
                m[i+53]<Modes.min_peak_amp         ||
                (float) m[i+2]/m[i+52] > Modes.diffratioclose ||
                (float) m[i+2]/m[i+53] > Modes.diffratioclose ||
                (float) m[i+51]/m[i+52] > Modes.diffratioclose ||
                (float) m[i+51]/m[i+53] > Modes.diffratioclose ||
                (float) m[i+54]/m[i+52] > Modes.diffratioclose ||
                (float) m[i+54]/m[i+53] > Modes.diffratioclose ||
                (float) m[i+55]/m[i+52] > Modes.diffratio ||
                (float) m[i+55]/m[i+53] > Modes.diffratio) { goto next_loop; }
            Modes.type = 2;
            os = 57;
        p4_check:

            /* Checks if Mode A or C message has short p4 pulse */
            if ((float) m[i+os-1]/m[i+os]<Modes.diffratioclosep4 && (float) m[i+os-1]/m[i+os+1]<Modes.diffratioclosep4
            && (float) m[i+os-2]/m[i+os]<Modes.diffratiop4      && (float) m[i+os-2]/m[i+os+1]<Modes.diffratiop4
            && (float) m[i+os-3]/m[i+os]<Modes.diffratioclosep4 && (float) m[i+os-3]/m[i+os+1]<Modes.diffratioclosep4
            && (float) m[i+os+2]/m[i+os]<Modes.diffratioclosep4 && (float) m[i+os+2]/m[i+os+1]<Modes.diffratioclosep4
            && (float) m[i+os+3]/m[i+os]<Modes.diffratiop4 && (float) m[i+os+3]/m[i+os+1]<Modes.diffratiop4
            && m[i+os] > Modes.min_peak_amp && m[i+os+1] > Modes.min_peak_amp
            && m[i+os+2] < Modes.max_noicefloor_close && m[i+os-1] < Modes.max_noicefloor_close
            && m[i+os-2] < Modes.max_noicefloor && m[i+os-3] < Modes.max_noicefloor_close)
            {
                if (Modes.print_detected == true)
                {
                    if (os == 25) {
                        printf("Mode A all-call Message in location: %d: ", i);
                        for(a = 0; a < os+5; a++) {
                            printf(" %d", m[i+a]);
                        }
                        printf("\n\n");
                    }
                    if (os == 57) {
                        printf("Mode C all-call Message in location: %d: ", i);
                        for(a = 0; a < os+5; a++) {
                            printf(" %d", m[i+a]);
                        }
                        printf("\n\n");
                    }
                }
                if (os == 25) { Modes.count_a_acac++; Modes.countm++; i += 27; }
                if (os == 57) { Modes.count_c_acac++; Modes.countm++; i += 59; }
                Modes.order[o] = 20 + Modes.type; /* 20 --> short p4 */
                o++;
            }

            /* Checks if Mode A or C message has long p4 pulse */
            else if ( (float) m[i+os-1]/m[i+os] < Modes.diffratioclosep4 && (float) m[i+os-1]/m[i+os+1] < Modes.diffratioclosep4
                  && (float) m[i+os-1]/m[i+os+2] < Modes.diffratioclosep4 && (float) m[i+os-1]/m[i+os+3] < Modes.diffratioclosep4
                  && (float) m[i+os-2]/m[i+os] < Modes.diffratiop4 && (float) m[i+os-2]/m[i+os+1] < Modes.diffratiop4
                  && (float) m[i+os-2]/m[i+os+2] < Modes.diffratiop4 && (float) m[i+os-2]/m[i+os+3] < Modes.diffratiop4
                  && (float) m[i+os-3]/m[i+os] < Modes.diffratioclosep4 && (float) m[i+os-3]/m[i+os+1] < Modes.diffratioclosep4
                  && (float) m[i+os-3]/m[i+os+2] < Modes.diffratioclosep4 && (float) m[i+os-3]/m[i+os+3] < Modes.diffratioclosep4
                  && (float) m[i+os+4]/m[i+os] < Modes.diffratioclosep4 && (float) m[i+os+4]/m[i+os+1] < Modes.diffratioclosep4
                  && (float) m[i+os+4]/m[i+os+2] < Modes.diffratioclosep4 && (float) m[i+os+4]/m[i+os+3] < Modes.diffratioclosep4
                  && m[i+os] > Modes.min_peak_amp && m[i+os+1] > Modes.min_peak_amp && m[i+os+2] > Modes.min_peak_amp && m[i+os+3] > Modes.min_peak_amp
                  && m[i+os-1] < Modes.max_noicefloor_close && m[i+os+4] < Modes.max_noicefloor_close && m[i+os-3] < Modes.max_noicefloor_close
                  && m[i+os-2] < Modes.max_noicefloor)
            {
                if (Modes.print_detected == true)
                {
                    if (os == 25) {
                        printf("Mode A all-call (Compatibility mode) Message starting from bit number: %d ", i);
                        for(a = 0; a < os+5; a++) {
                            printf(" %d", m[i+a]);
                        }
                        printf("\n\n");
                    }
                    if (os == 57) {
                        printf("Mode C all-call (Compatibility mode) Message starting from bit number: %d ", i);
                        for(a = 0; a < os+5; a++) {
                            printf(" %d", m[i+a]);
                        }
                        printf("\n\n");
                    }
                }
                if (os == 25) { Modes.count_a_acsac++; Modes.countm++; i += 29; }
                if (os == 57) { Modes.count_c_acsac++; Modes.countm++; i += 61; }
                Modes.order[o] = 30 + Modes.type; /* 30 --> long p4 (compatibility mode) */
                o++;
            }

            /* No p4 pulse */
            else
            {
                if (os == 25) {
                Modes.count_a++;
                Modes.countm++;
                    if (Modes.baselinemode == true)
                    {
                        for (a = i+3; a < 19; a++) {
                            averagenf.push_back(m[a]);
                        }
                    averagenfclose.push_back(m[i+19]);
                    averagenfclose.push_back(m[i+2]);
                    averagenfclose.push_back(m[i+22]);
                    averagenf.push_back(m[i+23]);
                    averagepulse.push_back(m[i]);
                    averagepulse.push_back(m[i+1]);
                    averagepulse.push_back(m[i+20]);
                    averagepulse.push_back(m[i+21]);
                    }
                    if (Modes.print_detected == true)
                    {
                        printf("Mode A Message starting from bit number: %d ", i);
                        for(a = 0; a < os+5; a++) {
                            printf(" %d", m[i+a]);
                        }
                        printf("\n\n");
                    }
                i += 23;
                Modes.order[o] = 10 + Modes.type;
                o++;
                }
                if (os == 57)
                {
                    Modes.count_c++;
                    Modes.countm++;
                    if (Modes.baselinemode == true)
                    {
                        for (c = i+3; c < 51; c++) {
                            averagenf.push_back(m[c]);
                        }
                        averagenfclose.push_back(m[i+51]);
                        averagenfclose.push_back(m[i+2]);
                        averagenfclose.push_back(m[i+54]);
                        averagepulse.push_back(m[i+52]);
                        averagepulse.push_back(m[i+53]);
                        averagepulse.push_back(m[i]);
                        averagepulse.push_back(m[i+1]);
                    }

                    if (Modes.print_detected == true)
                    {
                        printf("Mode C Message starting from bit number: %d ", i);
                        for(a = 0; a < os+5; a++) {
                            printf(" %d", m[i+a]);
                        }
                        printf("\n\n");
                    }
                i += 56;
                Modes.order[o] = 10 + Modes.type; /* 10 --> no p4 */
                o++;
                }
            }
        next_loop:
            continue;
        }

        /* Takes averages of each pulse and non pulse type in detected messages and outputs them. */
        if (Modes.baselinemode == true)
        {
            int sum = 0;
            /* Average of pulse values */
            if (averagepulse.size() > 2)
            {
                for(i = 0; i < averagepulse.size(); i++) {
                    sum += (int) averagepulse.at(i);
                }
                printf("Recommended minimum pulse amplitude (mpa): %d\n",  sum/averagepulse.size());
            }
            sum = 0;
            /* Average of noicefloor not next to pulse values */
            if (averagenf.size() > 2)
            {
                for(i = 0; i < averagenf.size(); i++) {
                    sum += (int) averagenf.at(i);
                }
                printf("Recommended maximum noice floor (mnf): %d\n",  sum/averagenf.size());
            }
            sum = 0;
            /* Average of noicefloor next to pulse values */
            if (averagenfclose.size() > 2)
            {
                for(i = 0; i < averagenfclose.size(); i++) {
                    sum += (int) averagenfclose.at(i);
                }
                printf("Recommended minimum close pulse proximity noice floor: %d\n", sum/averagenfclose.size());
            }
            return;
        }
    }
}

/* Prints help */
void showHelp(void) {
    printf("Commands:\n"
    "--device           Input rtl-sdr device index\n"
    "--file             Input location and full name of test file that is used instead of rtl-sdr device\n"
    "--gain             Input desired gain level for rtl-sdr device\n"
    "--agc              Enable automatic gain control by RTL-SDR device\n"
    "--diff             Add minimum amplitude difference between pulses and non pulses that not right next to pulses\n"
    "--diffclose        Add minimum amplitude difference between pulses and non pulses right next to pulses\n"
    "--diffratio        Add minimum ratio of non pulse amplitude divided by pulse amplitude for every non pulse value not right next to pulse\n"
    "--diffratioclose   Add minimum ratio of non pulse amplitude divided by pulse amplitude for every non pulse value right next to pulse\n"
    "--diffratiop4      Same as diffratio but only for p4 check (Mode a/c only all-call and Mode a/c/s all-call (Compatibility mode) messages)\n"
    "Compares only p4 pulse values with non pulse values that are after p3 but not next to pulse values.\n"
    "--diffratioclosep4 Same as diffratio but only for p4 check (Mode a/c only all-call and Mode a/c/s all-call (Compatibility mode) messages)\n"
    "Compares only p4 pulse values with non pulse values that are after p3 and next to pulse values.\n"
    "--msgs             Show every recognized messages location and amplitude values in the message\n"
    "--order            Print order of different SSR interrogation signals\n"
    "--mpa              Minimum accepted pulse amplitude when there should be a pulse\n"
    "--mnf              Maximum allowed noicefloor amplitude when there shouldn't be a pulse\n"
    "--mnfc             Maximum allowed noicefloor amplitude when there shouldn't be a pulse right next to pulse\n"
    "--size             Defines size of read message when using rtl-sdr. Must be at least 512 otherwise uses default size of 262 144.\n"
    "--blmode           Outputs baseline values for mpa, mnf and mnfc based on accepted averages.\n"
    "--print            Print all captured amplitude data.\n"
    "--continuous       Keeps detecting and reporting messages continuously. Size parameter sets update interval.\n"
    "--help             Show this help\n");
}

/* Turns I/Q data to positive amplitude values with help of magnitude table */
void computeMagnitudeVector(void) {
    uint8_t *m = Modes.magnitude;
    unsigned char *p = Modes.data;
    uint32_t j;
    for (j = 0; j < Modes.data_length; j += 2) {
        int i = p[j]-127;
        int q = p[j+1]-127;

        if (i < 0) i = -i;
        if (q < 0) q = -q;
        m[j/2] = Modes.maglut[i*129+q];
    }
}

void populateMagnitudeTable(void) {
    uint8_t i;
    uint8_t q;
    /* Fill all possible I/Q values to table which saves time and processing power
     * since program doesn't have to calculate same squareroots or round numbers
     *
     * We multiply it by 1.405 to utilize full resolution (0-255).
     */
    Modes.maglut = (uint8_t *) malloc(129*129*2);
        for (i = 0; i <= 128; i++) {
            for (q = 0; q <= 128; q++) {
                Modes.maglut[i*129+q] = round(sqrt(i*i+q*q)*1.405);
            }
        }
    }

/* Prints statistics of different detected message types. */
void printStats(void) {
    int i;
    int j;
    int consecutive;
    if (Modes.countm == 0)
    {
        if (Modes.continuous == false)
        {
            printf("No messages detected.");
        }
    }

    else
    {
        printf("Statistics of measured data with length of %d bits:\n"
        "Messages recognized in total:                          %d\n"
        "Mode A messages recognized:                                 %d\n"
        "Mode C messages recognized:                                 %d\n"
        "Mode A All-Call messages recognized:                        %d\n"
        "Mode C All-Call messages recognized:                        %d\n"
        "Mode A All-Call (Compatibility Mode) messages recognized:    %d\n"
        "Mode C All-Call (Compatibility Mode) messages recognized:   %d\n"
        "Mode S messages recognized:                                 %d\n\n", Modes.data_length, Modes.countm, Modes.count_a, Modes.count_c,
                                                                           Modes.count_a_acac, Modes.count_c_acac, Modes.count_a_acsac,
                                                                           Modes.count_c_acsac, Modes.count_s);
        if (Modes.continuous == true)
        {
            Modes.cumulative_countm += Modes.countm;
            Modes.cumulative_count_a += Modes.count_a;
            Modes.cumulative_count_c += Modes.count_c;
            Modes.cumulative_count_a_acac += Modes.count_a_acac;
            Modes.cumulative_count_c_acac += Modes.count_c_acac;
            Modes.cumulative_count_a_acsac += Modes.count_a_acsac;
            Modes.cumulative_count_c_acsac += Modes.count_c_acsac;
            Modes.cumulative_count_s += Modes.count_s;
            printf("Cumulative statistics so far:\n"
            "Mode messages recognized in total:                          %d\n"
            "Mode A messages recognized:                                 %d\n"
            "Mode C messages recognized:                                 %d\n"
            "Mode A All-Call messages recognized:                        %d\n"
            "Mode C All-Call messages recognized:                        %d\n"
            "Mode A All-Call (Compatibility Mode) messages recognized:    %d\n"
            "Mode C All-Call (Compatibility Mode) messages recognized:   %d\n"
            "Mode S messages recognized:                                 %d\n\n", Modes.cumulative_countm, Modes.cumulative_count_a, Modes.cumulative_count_c,
                                                                           Modes.cumulative_count_a_acac, Modes.cumulative_count_c_acac, Modes.cumulative_count_a_acsac,
                                                                           Modes.cumulative_count_c_acsac, Modes.cumulative_count_s);
        }
        Modes.countm = 0;
        Modes.count_a = 0;
        Modes.count_c = 0;
        Modes.count_a_acac = 0;
        Modes.count_c_acac = 0;
        Modes.count_a_acsac = 0;
        Modes.count_c_acsac = 0;
        Modes.count_s = 0;
    }

    /* Prints the order of received messages. Prints message type based on order number given in detectMode function.
    * Counts consecutive messages, prints the amount instead of printing them separately. */
    if (Modes.order[0] != 0 && Modes.print_order == true && Modes.continuous == false) {
        printf("Sequence of recognized modes in message:\n");
        for (i = 0; i < Modes.countm; i++) {
            if (Modes.order[i] == 32) {
                if (Modes.order[i+1] == 32) {
                    consecutive = 2;
                    for (j = i+2; j < Modes.countm; j++) {
                        if (Modes.order[j] == 32) { consecutive++; }
                        else { break; }
                    }
                    printf("%d Mode C All-Call (Compatibility Mode) messages in a row\n", consecutive);
                    i = j-1;
                }
                else { printf("Mode C All-Call (Compatibility Mode)\n"); }
            }
            else if (Modes.order[i] == 22) {
                if (Modes.order[i+1] == 22) {
                    consecutive = 2;
                    for (j = i+2; j < Modes.countm; j++) {
                        if (Modes.order[j] == 22) { consecutive++; }
                        else { break; }
                    }
                    printf("%d Mode C All-Call messages in a row\n", consecutive);
                    i = j-1;
                }
                else { printf("Mode C All-Call message\n"); }
            }
            else if (Modes.order[i] == 21) {
                if (Modes.order[i+1] == 21) {
                    consecutive = 2;
                    for (j = i+2; j < Modes.countm; j++) {
                        if (Modes.order[j] == 21) { consecutive++; }
                        else { break; }
                    }
                    printf("%d Mode A All-Call messages in a row\n", consecutive);
                    i = j-1;
                }
                else { printf("Mode A All-Call message\n"); }
            }
            else if (Modes.order[i] == 11) {
                if (Modes.order[i+1] == 11) {
                    consecutive = 2;
                    for (j = i+2; j < Modes.countm; j++) {
                        if (Modes.order[j] == 11) { consecutive++; }
                        else { break; }
                    }
                    printf("%d Mode A messages in a row\n", consecutive);
                    i = j-1;
                }
                else { printf("Mode A message\n"); }
            }
            else if (Modes.order[i] == 12) {
                if (Modes.order[i+1] == 12) {
                    consecutive = 2;
                    for (j = i+2; j < Modes.countm; j++) {
                        if (Modes.order[j] == 12) { consecutive++; }
                        else { break; }
                    }
                    printf("%d Mode C messages in a row\n", consecutive);
                    i = j-1;
                }
                else { printf("Mode C message\n"); }
            }
            else if (Modes.order[i] == 31) {
                if (Modes.order[i+1] == 31) {
                    consecutive = 2;
                    for (j = i+2; j < Modes.countm; j++) {
                        if (Modes.order[j] == 31) { consecutive++; }
                        else { break; }
                    }
                    printf("%d Mode A All-Call (Compatibility Mode) messages in a row\n", consecutive);
                    i = j-1;
                }
                else { printf("Mode A All-Call (Compatibility Mode) message\n"); }
            }
            else if (Modes.order[i] == 3) {
                if (Modes.order[i+1] == 3) {
                    consecutive = 2;
                    for (j = i+2; j < Modes.countm; j++) {
                        if (Modes.order[j] == 3) { consecutive++; }
                        else { break; }
                    }
                    printf("%d Mode S messages in a row\n", consecutive);
                    i = j-1;
                }
                else { printf("Mode S message\n"); }
            }
        }
    }
}

/* Reads data from file */
void readDataFromFile(void) {

        ssize_t nread, toread;

        Modes.fd = open(Modes.filename, O_RDONLY);
        unsigned char *p;
        toread = Modes.data_length;
        p = Modes.data;
        while(toread) {
            nread = read(Modes.fd, p, toread);
            if (nread <= 0) {
                break;
            }
            p += nread;
            toread -= nread;
        }
        Modes.data_ready = 1;
        pthread_cond_signal(&Modes.data_cond);
        pthread_mutex_unlock(&Modes.data_mutex);
    }

void rtlsdrCallback(unsigned char *buf, uint32_t len, void *ctx) {

    /* Read the new data. */
    memcpy(Modes.data, buf, Modes.data_length);
    pthread_cond_signal(&Modes.data_cond);
    pthread_mutex_unlock(&Modes.data_mutex);
    if (Modes.continuous == false)
    {
        rtlsdr_cancel_async(Modes.dev);
    }
    Modes.data_ready = true;
}

void *dataReader(void *arg) {
    if (Modes.filename == NULL) {
        modesInitRTLSDR();

        rtlsdr_read_async(Modes.dev, rtlsdrCallback, NULL,
                              MODES_ASYNC_BUF_NUMBER,
                              Modes.data_length);

    } else {
        readDataFromFile();
    }
    return NULL;
}


int main(int argc, char **argv) {
    int i;

    modesInit();
    populateMagnitudeTable();


    /* Read commandline options */
    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i],"--device")) {
            Modes.dev_index = atoi(argv[++i]);
        } else if (!strcmp(argv[i],"--dl")) {
            Modes.downlink = false;
            Modes.freq = 1090000000;
            Modes.samplerate = 2000000;
        } else if (!strcmp(argv[i],"--gain")) {
            Modes.gain = atof(argv[++i]);
        } else if (!strcmp(argv[i],"--size")) {
            if (atof(argv[++i]) >= 16384)
            {
                Modes.data_length = atof(argv[i]);
                Modes.data_length -= Modes.data_length%16384;
            }
        } else if (!strcmp(argv[i],"--file")) {
            Modes.filename = strdup(argv[++i]);
        } else if (!strcmp(argv[i],"--agc")) {
            Modes.enable_agc = 1;
        } else if (!strcmp(argv[i],"--diff")) {
            Modes.diff = atoi(argv[++i]);
        } else if (!strcmp(argv[i],"--diffratio")) {
            Modes.diffratio = atof(argv[++i]);
        } else if (!strcmp(argv[i],"--diffclose")) {
            Modes.diffclose = atoi(argv[++i]);
        } else if (!strcmp(argv[i],"--diffratioclose")) {
            Modes.diffratioclose = atof(argv[++i]);
        } else if (!strcmp(argv[i],"--diffratiop4")) {
            Modes.diffratiop4 = atof(argv[++i]);
        } else if (!strcmp(argv[i],"--diffratioclosep4")) {
            Modes.diffratioclosep4 = atof(argv[++i]);
        } else if (!strcmp(argv[i],"--blmode")) {
            Modes.baselinemode = true;
        } else if (!strcmp(argv[i],"--order")) {
            Modes.print_order = true;
        } else if (!strcmp(argv[i],"--msgs")) {
            Modes.print_detected = true;
        } else if (!strcmp(argv[i],"--mpa")) {
            Modes.min_peak_amp = atoi(argv[++i]);
        } else if (!strcmp(argv[i],"--mnf")) {
            Modes.max_noicefloor = atoi(argv[++i]);
        } else if (!strcmp(argv[i],"--mnfc")) {
            Modes.max_noicefloor_close = atoi(argv[++i]);
        } else if (!strcmp(argv[i],"--print")) {
            Modes.print_all = true;
        } else if (!strcmp(argv[i],"--continuous")) {
            Modes.continuous = true;
        } else if (!strcmp(argv[i],"--help")) {
            showHelp();
            exit(1);
        }
        else {
            printf("No commands recognized\n");
            showHelp();
            exit(1);
        }
    }

    dataInit();

    pthread_create(&Modes.reader_thread, NULL, dataReader, NULL);

    pthread_mutex_lock(&Modes.data_mutex);
    if (Modes.continuous == true)
    {
        while(1) {
            if (Modes.data_ready == false) {
                pthread_cond_wait(&Modes.data_cond,&Modes.data_mutex);
                continue;
            }

            computeMagnitudeVector();
            pthread_mutex_unlock(&Modes.data_mutex);
            Modes.data_ready = false;
            pthread_cond_signal(&Modes.data_cond);

            detectMode(Modes.magnitude);
            printStats();
            pthread_mutex_lock(&Modes.data_mutex);
        }
    }

    else
    {
        if (Modes.data_ready == false) {
            pthread_cond_wait(&Modes.data_cond,&Modes.data_mutex);
        }

        computeMagnitudeVector();
        pthread_mutex_unlock(&Modes.data_mutex);
        Modes.data_ready = false;
        pthread_cond_signal(&Modes.data_cond);
        detectMode(Modes.magnitude);
        printStats();
        pthread_mutex_lock(&Modes.data_mutex);
    }

    rtlsdr_close(Modes.dev);
    return 0;
}
