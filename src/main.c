#include "stm32f0xx.h"

void set_char_msg(int, char);
void nano_wait(unsigned int);
void game(void);
void internal_clock();
void check_wiring();
void autotest();


// Configure GPIOC
void enable_ports(void) {
    // Only enable port C for the keypad
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~0xffff;
    GPIOC->MODER |= 0x55 << (4*2);
    GPIOC->OTYPER &= ~0xff;
    GPIOC->OTYPER |= 0xf0;
    GPIOC->PUPDR &= ~0xff;
    GPIOC->PUPDR |= 0x55;
}


uint8_t col; // the column being scanned

void drive_column(int);   // energize one of the column outputs
int  read_rows();         // read the four row inputs
void update_history(int col, int rows); // record the buttons of the driven column
char get_key_event(void); // wait for a button event (press or release)
char get_keypress(void);  // wait for only a button press event.
float getfloat(void);     // read a floating-point number from keypad
void show_keys(void);     // demonstrate get_key_event()


// Bit Bang SPI LED Array
int msg_index = 0;
uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
extern const char font[];


// Configure PB12 (CS), PB13 (SCK), and PB15 (SDI) for outputs
void setup_bb(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    GPIOB->MODER &= ~0x45000000; // 01 (general purpose output)
    GPIOB->MODER |= 0x45000000;

    GPIOB->ODR |= 0x1000;
    GPIOB->ODR &= ~0x2000;
}


void small_delay(void) {
    nano_wait(50000);
}


// Set the MOSI bit, then set the clock high and low.
// Pause between doing these steps with small_delay().
void bb_write_bit(int val) {
    // CS (PB12)
    // SCK (PB13)
    // SDI (PB15)
    if(val) {
        GPIOB->ODR |= 0x8000;
    }
    else {
        GPIOB->ODR &= ~0x8000;;
    }
    small_delay();
    GPIOB->ODR |= 0x2000;
    small_delay();
    GPIOB->ODR &= ~0x2000;
}


// Set CS (PB12) low,
// write 16 bits using bb_write_bit,
// then set CS high.
void bb_write_halfword(int halfword) {
    GPIOB->ODR &= ~0x1000;
    bb_write_bit(halfword & 0x1);
    bb_write_bit(halfword & 0x2);
    bb_write_bit(halfword & 0x3);
    bb_write_bit(halfword & 0x4);
    bb_write_bit(halfword & 0x5);
    bb_write_bit(halfword & 0x6);
    bb_write_bit(halfword & 0x7);
    bb_write_bit(halfword & 0x8);
    bb_write_bit(halfword & 0x9);
    bb_write_bit(halfword & 0xA);
    bb_write_bit(halfword & 0xB);
    bb_write_bit(halfword & 0xC);
    bb_write_bit(halfword & 0xD);
    bb_write_bit(halfword & 0xE);
    bb_write_bit(halfword & 0xF);
    GPIOB->ODR |= 0x1000;
}


// Continually bitbang the msg[] array.
void drive_bb(void) {
    for(;;)
        for(int d=0; d<8; d++) {
            bb_write_halfword(msg[d]);
            nano_wait(1000000); // wait 1 ms between digits
        }
}


// Configure Timer 15 for an update rate of 1 kHz.
// Trigger the DMA channel on each update.
void init_tim15(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    TIM15->PSC = 4799; // 48 MHz -> 10 KHz
    TIM15->ARR = 9; // 10 KHz -> 1KHz
    TIM15->DIER |= TIM_DIER_UDE;
    // TIM15->DIER &= ~TIM_DIER_UIE;
    TIM15->CR1 |= TIM_CR1_CEN;
}


// Configure timer 7 to invoke the update interrupt at 1kHz
void init_tim7(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->PSC = 4799; // 48 MHz -> 10 KHz
    TIM7->ARR = 9; // 10 KHz -> 1KHz

    TIM7->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM7_IRQn);
    TIM7->CR1 |= TIM_CR1_CEN;
}


void TIM7_IRQHandler() {
    TIM7->SR &= ~TIM_SR_UIF; // acknowlegde interupt by clearing interrupt flag
  
    int rows = read_rows();
    update_history(col,rows);
    col = (col + 1) & 3;
    drive_column(col);
}


// Initialize the SPI2 peripheral.
void init_spi2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // set to AF mode (10) then AF0 for PB12/13/15
    GPIOB->MODER &= ~0x8A000000; 
    GPIOB->MODER |= 0x8A000000;
    GPIOB->AFR[1] &= ~0xF0FF0000;

    SPI2->CR1 &= ~SPI_CR1_SPE;
    SPI2->CR1 |= SPI_CR1_BR; // max size
    SPI2->CR1 |= SPI_CR1_MSTR;

    // SPI2->CR2 &= ~0xF00; // 16 bit size in CR2_DS
    SPI2->CR2 |= 0xF00;

    SPI2->CR2 |= SPI_CR2_SSOE; 
    SPI2->CR2 |= SPI_CR2_NSSP;
    SPI2->CR2 |= SPI_CR2_TXDMAEN;

    SPI2->CR1 |= SPI_CR1_SPE;
}


// Configure the SPI2 peripheral to trigger the DMA channel when the
// transmitter is empty.
void spi2_setup_dma(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    SPI2->CR2 |= SPI_CR2_TXDMAEN;

    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CMAR = msg; // figure out
    DMA1_Channel5->CPAR = &(SPI2->DR);
    DMA1_Channel5->CNDTR = 8;

    DMA1_Channel5->CCR |= DMA_CCR_DIR;
    DMA1_Channel5->CCR |= DMA_CCR_MINC;
    DMA1_Channel5->CCR |= DMA_CCR_MSIZE_0;
    DMA1_Channel5->CCR |= DMA_CCR_PSIZE_0;
    DMA1_Channel5->CCR |= DMA_CCR_CIRC;
}


// Enable the DMA channel.
void spi2_enable_dma(void) {
    DMA1_Channel5->CCR |= DMA_CCR_EN;
}


// 4.4 SPI OLED Display
void init_spi1() {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // set to AF mode (10) then AF0 for PA5/7/15
    GPIOA->MODER &= ~0x80008800; 
    GPIOA->MODER |= 0x80008800;
    GPIOA->AFR[1] &= ~0xF0000000;
    GPIOA->AFR[0] &= ~0xF0F00000;

    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 |= SPI_CR1_BR; // max size
    SPI1->CR1 |= SPI_CR1_MSTR;

    SPI1->CR2 &= ~SPI_CR2_DS; // sets to 0b0111
    SPI1->CR2 |= 0x800; // sets to 0b1111
    SPI1->CR2 &= ~0x600; // sets to 0b1001 (10 bits)

    SPI1->CR2 |= SPI_CR2_SSOE; 
    SPI1->CR2 |= SPI_CR2_NSSP;
    SPI1->CR2 |= SPI_CR2_TXDMAEN;

    SPI1->CR1 |= SPI_CR1_SPE;
}
void spi_cmd(unsigned int data) {   
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = data;
}
void spi_data(unsigned int data) {
    spi_cmd(data | 0x200);
}
void spi1_init_oled() {
    nano_wait(1000000);
    spi_cmd(0x38);
    spi_cmd(0x08);
    spi_cmd(0x01);
    nano_wait(2000000);
    spi_cmd(0x06);
    spi_cmd(0x02);
    spi_cmd(0x0C);
}
void spi1_display1(const char *string) {
    spi_cmd(0x02);

    while(*string != "\0") {
        spi_data(*string);
        string++;
    } 
}
void spi1_display2(const char *string) {
    spi_cmd(0xC0);

    while(*string != "\0") {
        spi_data(*string);
        string++;
    } 
}


// This is the 34-entry buffer to be copied into SPI1.
// Each element is a 16-bit value that is either character data or a command.
// Element 0 is the command to set the cursor to the first position of line 1.
// The next 16 elements are 16 characters.
// Element 17 is the command to set the cursor to the first position of line 2.
uint16_t display[34] = {
        0x002, // Command to set the cursor at the first position line 1
        0x200+'E', 0x200+'C', 0x200+'E', 0x200+'3', 0x200+'6', + 0x200+'2', 0x200+' ', 0x200+'i',
        0x200+'s', 0x200+' ', 0x200+'t', 0x200+'h', + 0x200+'e', 0x200+' ', 0x200+' ', 0x200+' ',
        0x0c0, // Command to set the cursor at the first position line 2
        0x200+'c', 0x200+'l', 0x200+'a', 0x200+'s', 0x200+'s', + 0x200+' ', 0x200+'f', 0x200+'o',
        0x200+'r', 0x200+' ', 0x200+'y', 0x200+'o', + 0x200+'u', 0x200+'!', 0x200+' ', 0x200+' ',
};


// Configure the proper DMA channel to be triggered by SPI1_TX.
// Set the SPI1 peripheral to trigger a DMA when the transmitter is empty.
void spi1_setup_dma(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    SPI1->CR2 |= SPI_CR2_TXDMAEN;

    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CMAR = display;
    DMA1_Channel3->CPAR = &(SPI1->DR);
    DMA1_Channel3->CNDTR = 8;
    DMA1_Channel3->CCR |= DMA_CCR_DIR;
    DMA1_Channel3->CCR |= DMA_CCR_MINC;
    DMA1_Channel3->CCR |= DMA_CCR_MSIZE_0;
    DMA1_Channel3->CCR |= DMA_CCR_PSIZE_0;
    DMA1_Channel3->CCR |= DMA_CCR_CIRC;

    // RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    // SPI2->CR2 |= SPI_CR2_TXDMAEN;

    // DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    // DMA1_Channel5->CMAR = msg;
    // DMA1_Channel5->CPAR = &(SPI2->DR);
    // DMA1_Channel5->CNDTR = 8;

    // DMA1_Channel5->CCR |= DMA_CCR_DIR;
    // DMA1_Channel5->CCR |= DMA_CCR_MINC;
    // DMA1_Channel5->CCR |= DMA_CCR_MSIZE_0;
    // DMA1_Channel5->CCR |= DMA_CCR_PSIZE_0;
    // DMA1_Channel5->CCR |= DMA_CCR_CIRC;
}


// Enable the DMA channel triggered by SPI1_TX.
void spi1_enable_dma(void) {
    DMA1_Channel3->CCR |= DMA_CCR_EN;
}


// Main function
int main(void) {
    internal_clock();

    msg[0] |= font['E'];
    msg[1] |= font['C'];
    msg[2] |= font['E'];
    msg[3] |= font[' '];
    msg[4] |= font['3'];
    msg[5] |= font['6'];
    msg[6] |= font['2'];
    msg[7] |= font[' '];

    autotest();

    // GPIO enable
    enable_ports();
    // setup keyboard
    init_tim7();

    // LED array Bit Bang
#define BIT_BANG
#if defined(BIT_BANG)
    setup_bb();
    drive_bb();
#endif

    // Direct SPI peripheral to drive LED display
//#define SPI_LEDS
#if defined(SPI_LEDS)
    init_spi2();
    spi2_setup_dma();
    spi2_enable_dma();
    init_tim15();
    show_keys();
#endif

    // LED array SPI
// #define SPI_LEDS_DMA
#if defined(SPI_LEDS_DMA)
    init_spi2();
    spi2_setup_dma();
    spi2_enable_dma();
    show_keys();
#endif

    // SPI OLED direct drive
//#define SPI_OLED
#if defined(SPI_OLED)
    init_spi1();
    spi1_init_oled();
    spi1_display1("Hello again,");
    spi1_display2(username);
#endif

    // SPI
//#define SPI_OLED_DMA
#if defined(SPI_OLED_DMA)
    init_spi1();
    spi1_init_oled();
    spi1_setup_dma();
    spi1_enable_dma();
#endif

    // Uncomment when you are ready to generate a code.
    // autotest();

    // Game on!  The goal is to score 100 points.
    game();
}
