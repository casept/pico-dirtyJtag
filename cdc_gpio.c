/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2026 Davids Paskevics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "dirtyJtagConfig.h"

#if CDC_GPIO_ENABLE

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/bootrom.h"
#include "tusb.h"

#include "cdc_gpio.h"

#define LINE_BUF_SIZE 128
#define HISTORY_SIZE 8

/* Alphabetised for tidy multi-match listings. */
static const char *const command_table[] = {
    "bootloader",
    "gpioclear",
    "gpioget",
    "gpiopulse",
    "gpioset",
    "help",
};
#define NUM_COMMANDS ((int)(sizeof(command_table) / sizeof(command_table[0])))

static char line_buf[LINE_BUF_SIZE];
static size_t line_len;
static bool was_connected;
static bool pulse_active;
static absolute_time_t pulse_end_time;
static uint pulse_pin;
/* Tracks the previous char if it was an EOL, so we can swallow the
 * complementary half of a CRLF / LFCR pair without dropping empty lines. */
static char prev_eol;

/* ANSI escape-sequence state for arrow keys. */
enum { ESC_NONE, ESC_SEEN, ESC_BRACKET };
static int esc_state;

/* History ring buffer. history_view == -1 means live input. */
static char history[HISTORY_SIZE][LINE_BUF_SIZE];
static size_t history_lens[HISTORY_SIZE];
static int history_count;
static int history_view;
static char draft_buf[LINE_BUF_SIZE];
static size_t draft_len;

static void flush_out(void)
{
    tud_cdc_n_write_flush(CDC_GPIO_ITF);
}

static void write_str(const char *s)
{
    tud_cdc_n_write(CDC_GPIO_ITF, s, strlen(s));
    flush_out();
}

static void write_char(char c)
{
    tud_cdc_n_write(CDC_GPIO_ITF, &c, 1);
    flush_out();
}

static void send_prompt(void)
{
    write_str("dirtyjtag> ");
}

/* Redraw the current input line: \r, prompt, content, clear-to-end. */
static void redraw_line(void)
{
    write_char('\r');
    send_prompt();
    if (line_len > 0)
    {
        tud_cdc_n_write(CDC_GPIO_ITF, line_buf, line_len);
        flush_out();
    }
    write_str("\x1b[K");
}

static int history_slot_for_view(int view)
{
    int avail = history_count < HISTORY_SIZE ? history_count : HISTORY_SIZE;
    if (view < 0 || view >= avail) return -1;
    int newest = (history_count - 1) % HISTORY_SIZE;
    return (newest - view + HISTORY_SIZE) % HISTORY_SIZE;
}

static void load_view(int view)
{
    if (view == -1)
    {
        line_len = draft_len;
        memcpy(line_buf, draft_buf, line_len);
    }
    else
    {
        int slot = history_slot_for_view(view);
        if (slot < 0) return;
        line_len = history_lens[slot];
        memcpy(line_buf, history[slot], line_len);
    }
    redraw_line();
}

static void handle_up(void)
{
    if (history_count == 0) { write_char('\a'); return; }
    int avail = history_count < HISTORY_SIZE ? history_count : HISTORY_SIZE;
    if (history_view == -1)
    {
        /* Save whatever is currently typed as the live draft. */
        draft_len = line_len;
        memcpy(draft_buf, line_buf, line_len);
        history_view = 0;
    }
    else if (history_view + 1 < avail)
    {
        history_view++;
    }
    else
    {
        write_char('\a');
        return;
    }
    load_view(history_view);
}

static void handle_down(void)
{
    if (history_view == -1) { write_char('\a'); return; }
    history_view--;
    load_view(history_view);
}

static void handle_tab(void)
{
    int matches = 0;
    int last_match = -1;
    for (int i = 0; i < NUM_COMMANDS; i++)
    {
        if (strncmp(command_table[i], line_buf, line_len) == 0)
        {
            matches++;
            last_match = i;
        }
    }
    if (matches == 0) { write_char('\a'); return; }
    if (matches == 1)
    {
        const char *cmd = command_table[last_match];
        size_t clen = strlen(cmd);
        for (size_t i = line_len; i < clen && line_len < LINE_BUF_SIZE - 1; i++)
        {
            line_buf[line_len++] = cmd[i];
            write_char(cmd[i]);
        }
        return;
    }
    /* Multiple matches: list candidates, then redraw the input. */
    write_str("\r\n");
    for (int i = 0; i < NUM_COMMANDS; i++)
    {
        if (strncmp(command_table[i], line_buf, line_len) == 0)
        {
            write_str("  ");
            write_str(command_table[i]);
            write_str("\r\n");
        }
    }
    redraw_line();
}

static void push_history(void)
{
    if (line_len == 0) return;
    int newest_slot = (history_count - 1 + HISTORY_SIZE) % HISTORY_SIZE;
    if (history_count > 0 &&
        history_lens[newest_slot] == line_len &&
        memcmp(history[newest_slot], line_buf, line_len) == 0)
    {
        return; /* skip duplicate of most-recent entry */
    }
    int slot = history_count % HISTORY_SIZE;
    memcpy(history[slot], line_buf, line_len);
    history_lens[slot] = line_len;
    history_count++;
}

static bool parse_pin(const char *s, uint *out)
{
    if (*s == '\0') return false;
    char *end;
    long v = strtol(s, &end, 10);
    if (*end != '\0' || v < 0 || v >= NUM_BANK0_GPIOS) return false;
    *out = (uint)v;
    return true;
}

static bool parse_duration(const char *s, uint32_t *out)
{
    if (*s == '\0') return false;
    char *end;
    long v = strtol(s, &end, 10);
    if (*end != '\0' || v < 0) return false;
    *out = (uint32_t)v;
    return true;
}

static void cmd_help(void)
{
    write_str(
        "Commands:\r\n"
        "  help\r\n"
        "  gpioget,<PIN>\r\n"
        "  gpioset,<PIN>\r\n"
        "  gpioclear,<PIN>\r\n"
        "  gpiopulse,<PIN>,<DURATION_MS>\r\n"
        "  bootloader\r\n"
        "Line editing: TAB completes, Up/Down recall history, BS erases, Ctrl-U kills line, Ctrl-C aborts.\r\n"
    );
}

static void cmd_gpioget(uint pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
    /* Let the pull settle a tick so floating pins read consistently. */
    busy_wait_us_32(2);
    int v = gpio_get(pin);
    gpio_disable_pulls(pin);
    char buf[16];
    snprintf(buf, sizeof(buf), "%d\r\n", v ? 1 : 0);
    write_str(buf);
}

static void cmd_gpioset(uint pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 1);
    write_str("OK\r\n");
}

static void cmd_gpioclear(uint pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
    write_str("OK\r\n");
}

static void cmd_gpiopulse(uint pin, uint32_t duration_ms)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 1);
    pulse_pin = pin;
    pulse_active = true;
    pulse_end_time = make_timeout_time_ms(duration_ms);
    write_str("OK\r\n");
}

static void cmd_bootloader(void)
{
    write_str("Entering BOOTSEL...\r\n");
    flush_out();
    /* Give the host a moment to receive the line before USB drops. */
    sleep_ms(50);
    reset_usb_boot(0, 0);
    /* not reached */
}

static void execute_line(void)
{
    line_buf[line_len] = '\0';

    char *cmd = line_buf;
    char *arg1 = strchr(cmd, ',');
    if (arg1) *arg1++ = '\0';
    char *arg2 = arg1 ? strchr(arg1, ',') : NULL;
    if (arg2) *arg2++ = '\0';

    if (strcmp(cmd, "help") == 0)
    {
        cmd_help();
    }
    else if (strcmp(cmd, "gpioget") == 0)
    {
        uint pin;
        if (!arg1) { write_str("ERROR: missing PIN\r\n"); return; }
        if (!parse_pin(arg1, &pin)) { write_str("ERROR: invalid PIN\r\n"); return; }
        cmd_gpioget(pin);
    }
    else if (strcmp(cmd, "gpioset") == 0)
    {
        uint pin;
        if (!arg1) { write_str("ERROR: missing PIN\r\n"); return; }
        if (!parse_pin(arg1, &pin)) { write_str("ERROR: invalid PIN\r\n"); return; }
        cmd_gpioset(pin);
    }
    else if (strcmp(cmd, "gpioclear") == 0)
    {
        uint pin;
        if (!arg1) { write_str("ERROR: missing PIN\r\n"); return; }
        if (!parse_pin(arg1, &pin)) { write_str("ERROR: invalid PIN\r\n"); return; }
        cmd_gpioclear(pin);
    }
    else if (strcmp(cmd, "gpiopulse") == 0)
    {
        uint pin;
        uint32_t dur;
        if (!arg1 || !arg2) { write_str("ERROR: usage gpiopulse,<PIN>,<DURATION_MS>\r\n"); return; }
        if (!parse_pin(arg1, &pin)) { write_str("ERROR: invalid PIN\r\n"); return; }
        if (!parse_duration(arg2, &dur)) { write_str("ERROR: invalid DURATION\r\n"); return; }
        cmd_gpiopulse(pin, dur);
    }
    else if (strcmp(cmd, "bootloader") == 0)
    {
        cmd_bootloader();
    }
    else if (cmd[0] == '\0')
    {
        /* empty line, no-op */
    }
    else
    {
        write_str("ERROR: unknown command (try 'help')\r\n");
    }
}

void cdc_gpio_task(void)
{
    /* Service any active pulse first so timing is not tied to CDC traffic. */
    if (pulse_active && time_reached(pulse_end_time))
    {
        gpio_put(pulse_pin, 0);
        pulse_active = false;
    }

    if (!tud_cdc_n_connected(CDC_GPIO_ITF))
    {
        if (was_connected)
        {
            was_connected = false;
            line_len = 0;
            prev_eol = 0;
            esc_state = ESC_NONE;
            history_view = -1;
        }
        return;
    }

    if (!was_connected)
    {
        was_connected = true;
        write_str("\r\npico-dirtyJtag GPIO control. Type 'help'.\r\n");
        send_prompt();
    }

    uint32_t avail = tud_cdc_n_available(CDC_GPIO_ITF);
    for (uint32_t i = 0; i < avail; i++)
    {
        char c;
        if (tud_cdc_n_read(CDC_GPIO_ITF, &c, 1) != 1) break;

        /* ANSI escape sequence handling: \x1b [ A/B/C/D for arrows. */
        if (esc_state == ESC_BRACKET)
        {
            esc_state = ESC_NONE;
            switch (c)
            {
                case 'A': handle_up(); continue;
                case 'B': handle_down(); continue;
                case 'C':
                case 'D': continue; /* ignore left/right for now */
                default:  continue; /* unknown CSI, drop */
            }
        }
        if (esc_state == ESC_SEEN)
        {
            if (c == '[' || c == 'O') { esc_state = ESC_BRACKET; continue; }
            esc_state = ESC_NONE;
            continue; /* drop the rest of an unknown escape */
        }
        if (c == 0x1b) { esc_state = ESC_SEEN; continue; }

        if (c == '\r' || c == '\n')
        {
            /* Swallow the second half of a CRLF or LFCR pair. */
            bool is_complement = (c == '\n' && prev_eol == '\r') ||
                                 (c == '\r' && prev_eol == '\n');
            if (is_complement)
            {
                prev_eol = 0;
                continue;
            }
            prev_eol = c;

            write_str("\r\n");
            if (line_len > 0)
            {
                push_history();
                execute_line();
                line_len = 0;
            }
            history_view = -1;
            send_prompt();
        }
        else if (c == '\t')
        {
            prev_eol = 0;
            handle_tab();
        }
        else if (c == '\b' || c == 0x7f)
        {
            prev_eol = 0;
            if (line_len > 0)
            {
                line_len--;
                write_str("\b \b");
            }
        }
        else if (c == 0x15) /* Ctrl-U: kill line */
        {
            prev_eol = 0;
            line_len = 0;
            redraw_line();
        }
        else if (c == 0x03) /* Ctrl-C: abort line */
        {
            prev_eol = 0;
            write_str("^C\r\n");
            line_len = 0;
            history_view = -1;
            send_prompt();
        }
        else if (c >= 0x20 && c < 0x7f)
        {
            prev_eol = 0;
            if (line_len < LINE_BUF_SIZE - 1)
            {
                line_buf[line_len++] = c;
                write_char(c); /* echo */
            }
            else
            {
                write_char('\a'); /* bell: buffer full */
            }
        }
        /* other control bytes ignored */
    }
}

#endif /* CDC_GPIO_ENABLE */
