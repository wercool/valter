#include "pwm.h"


int pwmFreqSet(unsigned int channel, unsigned int freq)
{
    /* in order to get maximum resolution, the pre-scaler must be set to
     * something like freq << 16.  However, the mimimum pre-scaled frequency
     * we can get is MCLK (48MHz), the minimum is MCLK/(1024*255) =
     * 48MHz/261120 = 183Hz */
    unsigned int overall_div;
    unsigned int presc_total;
    unsigned int cpre = 0;
    unsigned int cprd;

    if (freq > MCK)
        return -1;
    overall_div = MCK / freq;

    if (overall_div > 0x7fff) {
        // divisor is larger than half the maximum CPRD register, we
        // have to configure prescalers
        presc_total = overall_div >> 15;

        // find highest 2^n fitting in prescaler (highest bit set)
        cpre = fhs(presc_total);
        if (cpre > 0) {
            // subtract one, because of fhs semantics //
            cpre--;
        }
        cprd = overall_div / (1 << cpre);
    } else
        cprd = overall_div;

    AT91F_PWMC_CfgChannel(AT91C_BASE_PWMC, channel, cpre | AT91C_PWMC_CPOL, cprd, 2);

    return 0;
}

void pwmDutySetPercent(unsigned int channel, unsigned int duty)
{
    unsigned int tmp = AT91C_BASE_PWMC->PWMC_CH[channel].PWMC_CUPDR & 0xffff;

    tmp = tmp << 16;    /* extend value by 2^16 */
    tmp = tmp / 100;    /* tmp = 1 % of extended cprd */
    tmp = duty * tmp;   /* tmp = 'duty' % of extended cprd */
    tmp = tmp >> 16;    /* un-extend tmp (divide by 2^16) */

    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, channel, tmp);
}
