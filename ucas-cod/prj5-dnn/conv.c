#include "div.h"
#include "mul.h"
#include "perf_cnt.h"
#include "printf.h"
#include "trap.h"

#define FRAC_BIT 10

#define RD_ADDR    135106448
#define RD_SIZE_D0 1
#define RD_SIZE_D1 1
#define RD_SIZE_D2 28
#define RD_SIZE_D3 28

#define WEIGHT_ADDR    134217728
#define WEIGHT_SIZE_D0 20
#define WEIGHT_SIZE_D1 1
#define WEIGHT_SIZE_D2 5
#define WEIGHT_SIZE_D3 5

#define WR_ADDR    135108240
#define WR_SIZE_D0 1
#define WR_SIZE_D1 20
#define WR_SIZE_D2 12
#define WR_SIZE_D3 12

#define KERN_ATTR_CONV_PAD       0
#define KERN_ATTR_CONV_STRIDE    1
#define KERN_ATTR_POOL_PAD       0
#define KERN_ATTR_POOL_KERN_SIZE 2
#define KERN_ATTR_POOL_STRIDE    2

// MMIO register address of DNN accelerator
#define GPIO_START_ADDR 0x60030000
#define GPIO_DONE_ADDR  0x60030008

struct size_vec4 {
        unsigned d0;
        unsigned d1;
        unsigned d2;
        unsigned d3;
};

struct mem_addr {
        unsigned rd_addr;
        unsigned weight_addr;
        unsigned wr_addr;
};

int mul(short a, short b)
{
#ifndef USE_MUL
        int ans = mul_ll(a, b);
#else
        int ans = a * b;
#endif
        return ans;
}

struct mem_addr addr = {RD_ADDR, WEIGHT_ADDR, WR_ADDR};
struct size_vec4 rd_size = {RD_SIZE_D0, RD_SIZE_D1, RD_SIZE_D2, RD_SIZE_D3};
struct size_vec4 wr_size = {WR_SIZE_D0, WR_SIZE_D1, WR_SIZE_D2, WR_SIZE_D3};
struct size_vec4 weight_size = {WEIGHT_SIZE_D0, WEIGHT_SIZE_D1, WEIGHT_SIZE_D2, WEIGHT_SIZE_D3};

struct size_vec4 conv_size;

extern char _binary_data_result_bin_start[];
extern char _binary_data_result_bin_size[];

void convolution()
{
        short *in = (short *)addr.rd_addr;
        short *weight = (short *)addr.weight_addr;
        short *out = (short *)addr.wr_addr;

        unsigned output_offset = 0;
        unsigned input_offset = 0;

        unsigned input_fm_w = rd_size.d3;
        unsigned input_fm_h = rd_size.d2;

        unsigned pad = KERN_ATTR_CONV_PAD;
        unsigned pad_len = pad << 1;

        unsigned conv_out_w = rd_size.d3 - weight_size.d3 + pad_len;
        unsigned conv_out_h = rd_size.d2 - weight_size.d2 + pad_len;

        unsigned stride = KERN_ATTR_CONV_STRIDE;

        conv_out_w = div(conv_out_w, stride);
        conv_out_h = div(conv_out_h, stride);

        conv_out_w++;
        conv_out_h++;

        conv_size.d0 = wr_size.d0;
        conv_size.d1 = wr_size.d1;
        conv_size.d2 = conv_out_h;
        conv_size.d3 = conv_out_w;

        // TODO: Please add your implementation here
        int no, ni, y, x, ky, kx, ih, iw, stride_y, stride_x;
        short bias;

        int in_square = mul(input_fm_h, input_fm_w);
        int weight_square = 1 + mul(WEIGHT_SIZE_D2, WEIGHT_SIZE_D3);
        int weight_no_offset, weight_offset;
        int temp;
        for (no = 0; no < conv_size.d1; no++) {
                // weight[no];
                weight_no_offset = mul(no, mul(rd_size.d1, weight_square));
                // weight[no][0][0]
                bias = weight[weight_no_offset];
                for (ni = 0; ni < rd_size.d1; ni++) {
                        // in[ni]
                        int in_ni_offset = mul(ni, in_square);
                        // weight[no][ni] + 1
                        int weight_ni_offset = weight_no_offset + mul(ni, weight_square) + 1;
                        for (y = 0; y < conv_out_h; y++) {
                                stride_y = mul(stride, y);
                                stride_x = 0;
                                for (x = 0; x < conv_out_w; x++) {
                                        temp = 0;
                                        for (ky = 0; ky < WEIGHT_SIZE_D2; ky++) {
                                                ih = ky + stride_y - pad;
                                                // in[ni][ih]
                                                int in_line_offset = mul(ih, input_fm_w);
                                                // weight[no][ni][ky]
                                                int weight_line_offset = mul(ky, WEIGHT_SIZE_D3);
                                                for (kx = 0; kx < WEIGHT_SIZE_D3; kx++) {
                                                        iw = kx + stride_x - pad;
                                                        if (iw >= 0 && ih >= 0 && iw < input_fm_w && ih < input_fm_h) {
                                                                // in[ni][ih][iw]
                                                                input_offset = in_ni_offset + in_line_offset + iw;
                                                                // weight[no][ni][ky][kx]
                                                                weight_offset = weight_ni_offset + weight_line_offset + kx;
                                                                temp += mul(in[input_offset], weight[weight_offset]);
                                                        }
                                                }
                                        }
                                        out[output_offset] = (temp >> FRAC_BIT) + bias;
                                        output_offset++;
                                        stride_x += stride;
                                }
                        }
                }
        }
}

void pooling()
{
        short *out = (short *)addr.wr_addr;

        unsigned output_offset = 0;
        unsigned input_offset = 0;

        unsigned input_fm_w = conv_size.d3;
        unsigned input_fm_h = conv_size.d2;

        unsigned pad = KERN_ATTR_POOL_PAD;
        unsigned pad_len = pad << 1;

        unsigned pad_w_test = conv_size.d3 - KERN_ATTR_POOL_KERN_SIZE;
        unsigned pad_h_test = conv_size.d2 - KERN_ATTR_POOL_KERN_SIZE;

        unsigned pool_out_w = pad_w_test + pad_len;
        unsigned pool_out_h = pad_h_test + pad_len;

        unsigned stride = KERN_ATTR_POOL_STRIDE;

        unsigned pad_w_test_remain = pad_w_test - mul(div(pad_w_test, stride), stride);
        unsigned pad_h_test_remain = pad_h_test - mul(div(pad_h_test, stride), stride);

        pool_out_w = div(pool_out_w, stride);
        pool_out_h = div(pool_out_h, stride);
        pool_out_w++;
        pool_out_h++;

        if ((!pad) && (pad_w_test_remain || pad_h_test_remain)) {
                pool_out_w++;
                pool_out_h++;
        }

        // TODO: Please add your implementation here
        int in_square = mul(input_fm_h, input_fm_w);
        int y, x, ky, kx, no, iw, ih;
        short max, value;
        int stride_y, stride_x, in_line_offset, input_no_offset;
        for (no = 0; no < conv_size.d1; no++) {
                input_no_offset = mul(no, in_square);
                for (y = 0; y < pool_out_h; y++) {
                        stride_y = mul(y, stride);
                        stride_x = 0;
                        for (x = 0; x < pool_out_w; x++) {
                                max = 1 << 15;
                                for (ky = 0; ky < KERN_ATTR_POOL_KERN_SIZE; ky++) {
                                        ih = ky + stride_y - pad;
                                        in_line_offset = mul(ih, input_fm_w);
                                        for (kx = 0; kx < KERN_ATTR_POOL_KERN_SIZE; kx++) {
                                                iw = kx + stride_x - pad;
                                                if (iw < 0 || ih < 0 || ih >= input_fm_h || iw >= input_fm_w) {
                                                        value = 0;
                                                } else {
                                                        input_offset = input_no_offset + in_line_offset + iw;
                                                        value = out[input_offset];
                                                }
                                                max = (max < value) ? value : max;
                                        }
                                }
                                out[output_offset] = max;
                                output_offset++;
                                stride_x += stride;
                        }
                }
        }
}

#ifdef USE_HW_ACCEL
void launch_hw_accel()
{
        volatile int *gpio_start = (void *)(GPIO_START_ADDR);
        volatile int *gpio_done = (void *)(GPIO_DONE_ADDR);

        // TODO: Please add your implementation here
        *gpio_start |= 0x1;
        while (!(*gpio_done) & 0x1)
                ;
}
#endif

int comparing()
{
        char *out = (char *)addr.wr_addr;
        char *result = (char *)_binary_data_result_bin_start;

#ifdef USE_HW_ACCEL
        int count = (int)_binary_data_result_bin_size +
                    (16 - WR_SIZE_D3) * 2 * WR_SIZE_D2 * WR_SIZE_D1;
#else
        int count = (int)_binary_data_result_bin_size;
#endif

        for (int i = 0, j = 0; i < count; i++) {
#ifdef USE_HW_ACCEL
                int alignment = i & 0x0000001f;
                if (alignment >= (WR_SIZE_D3 << 1))
                        continue;
#endif
                if (*(out + i) != *(result + j)) {
                        printf("Failed! at address %x and %x with data %x and %x\n", out + i, result + j, *(out + i), *(result + j));
                        return 1;
                }
                j++;
        }

        printf("Passed!\n");
        return 0;
}

int main()
{
        Result res;
        bench_prepare(&res);

#ifdef USE_HW_ACCEL
        printf("Launching task...\n");
        launch_hw_accel();
#else
        printf("starting convolution\n");
        convolution();
        printf("starting pooling\n");
        pooling();
#endif

        bench_done(&res);
        int result = comparing();
        printf("--------------------------------------");
        printf("total cycle      : %u\n", res.msec);
        printf("IW cycle         : %u\n", res.IW_cycles);
        printf("jump times       : %u\n", res.jmp_times);
        printf("branch times     : %u\n", res.branch_times);
        printf("branchValid times: %u\n", res.branchValid_times);
        printf("memRead cycle    : %u\n", res.memR_cycles);
        printf("memWrite cycle   : %u\n", res.memW_cycles);
        printf("instr num        : %u\n", res.instr_num);

        printf("benchmark finished\n");

        if (result == 0) {
                hit_good_trap();
        } else {
                nemu_assert(0);
        }

        return 0;
}
