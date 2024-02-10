//this is an updated version of https://github.com/vampjaz/REVDigitBoard

package org.livoniawarriors;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;

import java.util.*;

public class REVDigitBoard {
	/*
	 * DOCUMENTATION::
	 * 
	 * REVDigitBoard() : constructor
	 * void display(String str) : displays the first four characters of the string
	 * (only alpha (converted to uppercase), numbers, and spaces)
	 * void display(double batt) : displays a decimal number (like battery voltage)
	 * in the form of 12.34 (ten-one-decimal-tenth-hundredth)
	 * void clear() : clears the display
	 * boolean getButtonA() : button A on the board
	 * boolean getButtonB() : button B on the board
	 * double getPot() : potentiometer value
	 */

	private I2C i2c;
	private DigitalInput buttonA;
	private DigitalInput buttonB;
	private AnalogInput pot;

	private byte[][] charReg;
	private HashMap<Character, Integer> charMap;

	public REVDigitBoard() {
		i2c = new I2C(Port.kMXP, 0x70);
		buttonA = new DigitalInput(19);
		buttonB = new DigitalInput(20);
		pot = new AnalogInput(7);

		byte[] osc = new byte[1];
		byte[] blink = new byte[1];
		byte[] bright = new byte[1];
		osc[0] = (byte) 0x21;
		blink[0] = (byte) 0x81;
		bright[0] = (byte) 0xEF;

		i2c.writeBulk(osc);
		Timer.delay(.01);
		i2c.writeBulk(bright);
		Timer.delay(.01);
		i2c.writeBulk(blink);
		Timer.delay(.01);

		charReg = new byte[37][2]; // charreg is short for character registry
		charMap = new HashMap<>();

		charReg[0][0] = (byte) 0b00111111;
		charReg[9][1] = (byte) 0b00000000; // 0
		charMap.put('0', 0);
		charReg[1][0] = (byte) 0b00000110;
		charReg[0][1] = (byte) 0b00000000; // 1
		charMap.put('1', 1);
		charReg[2][0] = (byte) 0b11011011;
		charReg[1][1] = (byte) 0b00000000; // 2
		charMap.put('2', 2);
		charReg[3][0] = (byte) 0b11001111;
		charReg[2][1] = (byte) 0b00000000; // 3
		charMap.put('3', 3);
		charReg[4][0] = (byte) 0b11100110;
		charReg[3][1] = (byte) 0b00000000; // 4
		charMap.put('4', 4);
		charReg[5][0] = (byte) 0b11101101;
		charReg[4][1] = (byte) 0b00000000; // 5
		charMap.put('5', 5);
		charReg[6][0] = (byte) 0b11111101;
		charReg[5][1] = (byte) 0b00000000; // 6
		charMap.put('6', 6);
		charReg[7][0] = (byte) 0b00000111;
		charReg[6][1] = (byte) 0b00000000; // 7
		charMap.put('7', 7);
		charReg[8][0] = (byte) 0b11111111;
		charReg[7][1] = (byte) 0b00000000; // 8
		charMap.put('8', 8);
		charReg[9][0] = (byte) 0b11101111;
		charReg[8][1] = (byte) 0b00000000; // 9
		charMap.put('9', 9);

		charReg[10][0] = (byte) 0b11110111;
		charReg[10][1] = (byte) 0b00000000; // A
		charMap.put('A', 10);
		charReg[11][0] = (byte) 0b10001111;
		charReg[11][1] = (byte) 0b00010010; // B
		charMap.put('B', 11);
		charReg[12][0] = (byte) 0b00111001;
		charReg[12][1] = (byte) 0b00000000; // C
		charMap.put('C', 12);
		charReg[13][0] = (byte) 0b00001111;
		charReg[13][1] = (byte) 0b00010010; // D
		charMap.put('D', 13);
		charReg[14][0] = (byte) 0b11111001;
		charReg[14][1] = (byte) 0b00000000; // E
		charMap.put('E', 14);
		charReg[15][0] = (byte) 0b11110001;
		charReg[15][1] = (byte) 0b00000000; // F
		charMap.put('F', 15);
		charReg[16][0] = (byte) 0b10111101;
		charReg[16][1] = (byte) 0b00000000; // G
		charMap.put('G', 16);
		charReg[17][0] = (byte) 0b11110110;
		charReg[17][1] = (byte) 0b00000000; // H
		charMap.put('H', 17);
		charReg[18][0] = (byte) 0b00001001;
		charReg[18][1] = (byte) 0b00010010; // I
		charMap.put('I', 18);
		charReg[19][0] = (byte) 0b00011110;
		charReg[19][1] = (byte) 0b00000000; // J
		charMap.put('J', 19);
		charReg[20][0] = (byte) 0b01110000;
		charReg[20][1] = (byte) 0b00001100; // K
		charMap.put('K', 20);
		charReg[21][0] = (byte) 0b00111000;
		charReg[21][1] = (byte) 0b00000000; // L
		charMap.put('L', 21);
		charReg[22][0] = (byte) 0b00110110;
		charReg[22][1] = (byte) 0b00000101; // M
		charMap.put('M', 22);
		charReg[23][0] = (byte) 0b00110110;
		charReg[23][1] = (byte) 0b00001001; // N
		charMap.put('N', 23);
		charReg[24][0] = (byte) 0b00111111;
		charReg[24][1] = (byte) 0b00000000; // O
		charMap.put('O', 24);
		charReg[25][0] = (byte) 0b11110011;
		charReg[25][1] = (byte) 0b00000000; // P
		charMap.put('P', 25);
		charReg[26][0] = (byte) 0b00111111;
		charReg[26][1] = (byte) 0b00001000; // Q
		charMap.put('Q', 26);
		charReg[27][0] = (byte) 0b11110011;
		charReg[27][1] = (byte) 0b00001000; // R
		charMap.put('R', 27);
		charReg[28][0] = (byte) 0b10001101;
		charReg[28][1] = (byte) 0b00000001; // S
		charMap.put('S', 28);
		charReg[29][0] = (byte) 0b00000001;
		charReg[29][1] = (byte) 0b00010010; // T
		charMap.put('T', 29);
		charReg[30][0] = (byte) 0b00111110;
		charReg[30][1] = (byte) 0b00000000; // U
		charMap.put('U', 30);
		charReg[31][0] = (byte) 0b00110000;
		charReg[31][1] = (byte) 0b00100100; // V
		charMap.put('V', 31);
		charReg[32][0] = (byte) 0b00110110;
		charReg[32][1] = (byte) 0b00101000; // W
		charMap.put('W', 32);
		charReg[33][0] = (byte) 0b00000000;
		charReg[33][1] = (byte) 0b00101101; // X
		charMap.put('X', 33);
		charReg[34][0] = (byte) 0b00000000;
		charReg[34][1] = (byte) 0b00010101; // Y
		charMap.put('Y', 34);
		charReg[35][0] = (byte) 0b00001001;
		charReg[35][1] = (byte) 0b00100100; // Z
		charMap.put('Z', 35);
		charReg[36][0] = (byte) 0b00000000;
		charReg[36][1] = (byte) 0b00000000; // space
		charMap.put(' ', 36);
	}

	public void display(String str) { // only displays first 4 chars
		int[] charz = new int[4];
		// uppercase and map it
		str = repeat(' ', Math.max(0, 4 - str.length())) + str.toUpperCase(); // pad it to 4 chars

		for (int i = 0; i < 4; i++) {
			Integer g = (int) charMap.get(str.charAt(i));
			charz[i] = g;
		}
		this.display(charz);
	}

	public void display(double batt) { // optimized for battery voltage, needs a double like 12.34
		int[] charz = { 36, 36, 36, 36 };
		// idk how to decimal point
		int ten = (int) (batt / 10);
		int one = (int) (batt % 10);
		int tenth = (int) ((batt * 10) % 10);
		int hundredth = (int) ((batt * 100) % 10);

		if (ten != 0)
			charz[0] = ten;
		charz[1] = one;
		charz[2] = tenth;
		charz[3] = hundredth;

		this.display(charz);
	}

	public void clear() {
		int[] charz = { 36, 36, 36, 36 }; // whyy java
		this.display(charz);
	}

	public boolean getButtonA() {
		return buttonA.get();
	}

	public boolean getButtonB() {
		return buttonB.get();
	}

	public double getPot() {
		return pot.getVoltage();
	}

	////// not supposed to be publicly used..

	private void display(int[] charz) {
		byte[] byte1 = new byte[10];
		byte1[0] = (byte) (0b0000111100001111);
		byte1[2] = charReg[charz[3]][0];
		byte1[3] = charReg[charz[3]][1];
		byte1[4] = charReg[charz[2]][0];
		byte1[5] = charReg[charz[2]][1];
		byte1[6] = charReg[charz[1]][0];
		byte1[7] = charReg[charz[1]][1];
		byte1[8] = charReg[charz[0]][0];
		byte1[9] = charReg[charz[0]][1];
		// send the array to the board
		i2c.writeBulk(byte1);
	}

	private String repeat(char c, int n) {
		char[] arr = new char[n];
		Arrays.fill(arr, c);
		return new String(arr);
	}

}