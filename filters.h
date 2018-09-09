/* Copyright (C) 2018 Kristian Sloth Lauszus. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Sloth Lauszus
 Web      :  http://www.lauszus.com
 e-mail   :  lauszus@gmail.com
*/

#ifndef __FILTERS_H__
#define __FILTERS_H__

#if defined(__cplusplus)
extern "C" {
#endif

typedef struct {
    float Fc; /*!< Cutoff frequency in Hz */
    float prevOutput; /*!< The previous output from the filter */
} low_pass_t;

typedef struct {
    float Fc; /*!< Cutoff frequency in Hz */
    float prevOutput; /*!< The previous output from the filter */
    float prevInput; /*!< The value of the previous input to the filter */
} high_pass_t;

/**
 * Approximates first order low-pass filter: H(s) = 1/(tau*s + 1).
 * See: https://en.wikipedia.org/wiki/Exponential_smoothing.
 * Also read: http://techteach.no/simview/lowpass_filter/doc/filter_algorithm.pdf
 * @param low_pass  Pointer to ::low_pass_t structure.
 * @param input     The input to be filtered.
 * @param dt        The time between last filter time
 * @return          Returns the filtered value.
 */
float FILTERS_ApplyLowPass(low_pass_t *low_pass, float input, float dt);

/**
 * Approximates fist order high-pass filter: H(s) = (tau*s)/(tau*s + 1).
 * See: https://en.wikipedia.org/wiki/High-pass_filter.
 * @param high_pass Pointer to ::high_pass_t structure.
 * @param input     The input to be filtered.
 * @param dt        The time between last filter time
 * @return          Returns the filtered value.
 */
float FILTERS_ApplyHighPass(high_pass_t *high_pass, float input, float dt);

#if defined(__cplusplus)
}
#endif

#endif /* __FILTERS_H__ */
