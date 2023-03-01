package frc.robot.classes;

import java.util.HashMap;

public class ChooChooMap{
    private HashMap<Double, Double> chooChooMap = new HashMap<Double,Double>();

    public double toChooChooAngle(double armAngle){
        return chooChooMap.get(truncate(armAngle, 1));
    }

    private double truncate(double a, int places){
        return Math.round(a * Math.pow(10, places)) / Math.pow(10, places);
    }

    public ChooChooMap(){
        chooChooMap.put(90.7, 56.0);
        chooChooMap.put(90.6, 57.0);
        chooChooMap.put(90.5, 61.1);
        chooChooMap.put(90.4, 63.0);
        chooChooMap.put(90.3, 64.5);
        chooChooMap.put(90.2, 65.8);
        chooChooMap.put(90.1, 66.9);
        chooChooMap.put(90.0, 68.0);
        chooChooMap.put(89.9, 68.9);
        chooChooMap.put(89.8, 69.8);
        chooChooMap.put(89.7, 70.7);
        chooChooMap.put(89.6, 71.5);
        chooChooMap.put(89.5, 72.3);
        chooChooMap.put(89.4, 73.0);
        chooChooMap.put(89.3, 73.8);
        chooChooMap.put(89.2, 74.4);
        chooChooMap.put(89.1, 75.1);
        chooChooMap.put(89.0, 75.8);
        chooChooMap.put(88.9, 76.4);
        chooChooMap.put(88.8, 77.0);
        chooChooMap.put(88.7, 77.7);
        chooChooMap.put(88.6, 78.3);
        chooChooMap.put(88.5, 78.8);
        chooChooMap.put(88.4, 79.4);
        chooChooMap.put(88.3, 80.0);
        chooChooMap.put(88.2, 80.5);
        chooChooMap.put(88.1, 81.1);
        chooChooMap.put(88.0, 81.6);
        chooChooMap.put(87.9, 82.1);
        chooChooMap.put(87.8, 82.7);
        chooChooMap.put(87.7, 83.2);
        chooChooMap.put(87.6, 83.7);
        chooChooMap.put(87.5, 84.2);
        chooChooMap.put(87.4, 84.7);
        chooChooMap.put(87.3, 85.2);
        chooChooMap.put(87.2, 85.7);
        chooChooMap.put(87.1, 86.1);
        chooChooMap.put(87.0, 86.6);
        chooChooMap.put(86.9, 87.1);
        chooChooMap.put(86.8, 87.5);
        chooChooMap.put(86.7, 88.0);
        chooChooMap.put(86.6, 88.5);
        chooChooMap.put(86.5, 88.9);
        chooChooMap.put(86.4, 89.4);
        chooChooMap.put(86.3, 89.8);
        chooChooMap.put(86.2, 90.2);
        chooChooMap.put(86.1, 90.7);
        chooChooMap.put(86.0, 91.1);
        chooChooMap.put(85.9, 91.5);
        chooChooMap.put(85.8, 92.0);
        chooChooMap.put(85.7, 92.4);
        chooChooMap.put(85.6, 92.8);
        chooChooMap.put(85.5, 93.2);
        chooChooMap.put(85.4, 93.6);
        chooChooMap.put(85.3, 94.1);
        chooChooMap.put(85.2, 94.5);
        chooChooMap.put(85.1, 94.9);
        chooChooMap.put(85.0, 95.3);
        chooChooMap.put(84.9, 95.7);
        chooChooMap.put(84.8, 96.1);
        chooChooMap.put(84.7, 96.5);
        chooChooMap.put(84.6, 96.9);
        chooChooMap.put(84.5, 97.3);
        chooChooMap.put(84.4, 97.7);
        chooChooMap.put(84.3, 98.1);
        chooChooMap.put(84.2, 98.5);
        chooChooMap.put(84.1, 98.8);
        chooChooMap.put(84.0, 99.2);
        chooChooMap.put(83.9, 99.6);
        chooChooMap.put(83.8, 100.0);
        chooChooMap.put(83.7, 100.4);
        chooChooMap.put(83.6, 100.8);
        chooChooMap.put(83.5, 101.1);
        chooChooMap.put(83.4, 101.5);
        chooChooMap.put(83.3, 101.9);
        chooChooMap.put(83.2, 102.3);
        chooChooMap.put(83.1, 102.6);
        chooChooMap.put(83.0, 103.0);
        chooChooMap.put(82.9, 103.4);
        chooChooMap.put(82.8, 103.7);
        chooChooMap.put(82.7, 104.1);
        chooChooMap.put(82.6, 104.5);
        chooChooMap.put(82.5, 104.8);
        chooChooMap.put(82.4, 105.2);
        chooChooMap.put(82.3, 105.6);
        chooChooMap.put(82.2, 105.9);
        chooChooMap.put(82.1, 106.3);
        chooChooMap.put(82.0, 106.7);
        chooChooMap.put(81.9, 107.0);
        chooChooMap.put(81.8, 107.4);
        chooChooMap.put(81.7, 107.7);
        chooChooMap.put(81.6, 108.1);
        chooChooMap.put(81.5, 108.4);
        chooChooMap.put(81.4, 108.8);
        chooChooMap.put(81.3, 109.1);
        chooChooMap.put(81.2, 109.5);
        chooChooMap.put(81.1, 109.8);
        chooChooMap.put(81.0, 110.2);
        chooChooMap.put(80.9, 110.5);
        chooChooMap.put(80.8, 110.9);
        chooChooMap.put(80.7, 111.2);
        chooChooMap.put(80.6, 111.6);
        chooChooMap.put(80.5, 111.9);
        chooChooMap.put(80.4, 112.3);
        chooChooMap.put(80.3, 112.6);
        chooChooMap.put(80.2, 113.0);
        chooChooMap.put(80.1, 113.3);
        chooChooMap.put(80.0, 113.6);
        chooChooMap.put(79.9, 114.0);
        chooChooMap.put(79.8, 114.3);
        chooChooMap.put(79.7, 114.7);
        chooChooMap.put(79.6, 115.0);
        chooChooMap.put(79.5, 115.3);
        chooChooMap.put(79.4, 115.7);
        chooChooMap.put(79.3, 116.0);
        chooChooMap.put(79.2, 116.4);
        chooChooMap.put(79.1, 116.7);
        chooChooMap.put(79.0, 117.0);
        chooChooMap.put(78.9, 117.4);
        chooChooMap.put(78.8, 117.7);
        chooChooMap.put(78.7, 118.0);
        chooChooMap.put(78.6, 118.4);
        chooChooMap.put(78.5, 118.7);
        chooChooMap.put(78.4, 119.0);
        chooChooMap.put(78.3, 119.4);
        chooChooMap.put(78.2, 119.7);
        chooChooMap.put(78.1, 120.0);
        chooChooMap.put(78.0, 120.4);
        chooChooMap.put(77.9, 120.7);
        chooChooMap.put(77.8, 121.0);
        chooChooMap.put(77.7, 121.4);
        chooChooMap.put(77.6, 121.7);
        chooChooMap.put(77.5, 122.0);
        chooChooMap.put(77.4, 122.4);
        chooChooMap.put(77.3, 122.7);
        chooChooMap.put(77.2, 123.0);
        chooChooMap.put(77.1, 123.3);
        chooChooMap.put(77.0, 123.7);
        chooChooMap.put(76.9, 124.0);
        chooChooMap.put(76.8, 124.3);
        chooChooMap.put(76.7, 124.7);
        chooChooMap.put(76.6, 125.0);
        chooChooMap.put(76.5, 125.3);
        chooChooMap.put(76.4, 125.6);
        chooChooMap.put(76.3, 126.0);
        chooChooMap.put(76.2, 126.3);
        chooChooMap.put(76.1, 126.6);
        chooChooMap.put(76.0, 126.9);
        chooChooMap.put(75.9, 127.3);
        chooChooMap.put(75.8, 127.6);
        chooChooMap.put(75.7, 127.9);
        chooChooMap.put(75.6, 128.2);
        chooChooMap.put(75.5, 128.6);
        chooChooMap.put(75.4, 128.9);
        chooChooMap.put(75.3, 129.2);
        chooChooMap.put(75.2, 129.5);
        chooChooMap.put(75.1, 129.9);
        chooChooMap.put(75.0, 130.2);
        chooChooMap.put(74.9, 130.5);
        chooChooMap.put(74.8, 130.8);
        chooChooMap.put(74.7, 131.2);
        chooChooMap.put(74.6, 131.5);
        chooChooMap.put(74.5, 131.8);
        chooChooMap.put(74.4, 132.1);
        chooChooMap.put(74.3, 132.5);
        chooChooMap.put(74.2, 132.8);
        chooChooMap.put(74.1, 133.1);
        chooChooMap.put(74.0, 133.4);
        chooChooMap.put(73.9, 133.7);
        chooChooMap.put(73.8, 134.1);
        chooChooMap.put(73.7, 134.4);
        chooChooMap.put(73.6, 134.7);
        chooChooMap.put(73.5, 135.0);
        chooChooMap.put(73.4, 135.4);
        chooChooMap.put(73.3, 135.7);
        chooChooMap.put(73.2, 136.0);
        chooChooMap.put(73.1, 136.3);
        chooChooMap.put(73.0, 136.6);
        chooChooMap.put(72.9, 137.0);
        chooChooMap.put(72.8, 137.3);
        chooChooMap.put(72.7, 137.6);
        chooChooMap.put(72.6, 137.9);
        chooChooMap.put(72.5, 138.2);
        chooChooMap.put(72.4, 138.6);
        chooChooMap.put(72.3, 138.9);
        chooChooMap.put(72.2, 139.2);
        chooChooMap.put(72.1, 139.5);
        chooChooMap.put(72.0, 139.9);
        chooChooMap.put(71.9, 140.2);
        chooChooMap.put(71.8, 140.5);
        chooChooMap.put(71.7, 140.8);
        chooChooMap.put(71.6, 141.1);
        chooChooMap.put(71.5, 141.5);
        chooChooMap.put(71.4, 141.8);
        chooChooMap.put(71.3, 142.1);
        chooChooMap.put(71.2, 142.4);
        chooChooMap.put(71.1, 142.8);
        chooChooMap.put(71.0, 143.1);
        chooChooMap.put(70.9, 143.4);
        chooChooMap.put(70.8, 143.7);
        chooChooMap.put(70.7, 144.0);
        chooChooMap.put(70.6, 144.4);
        chooChooMap.put(70.5, 144.7);
        chooChooMap.put(70.4, 145.0);
        chooChooMap.put(70.3, 145.3);
        chooChooMap.put(70.2, 145.7);
        chooChooMap.put(70.1, 146.0);
        chooChooMap.put(70.0, 146.3);
        chooChooMap.put(69.9, 146.6);
        chooChooMap.put(69.8, 147.0);
        chooChooMap.put(69.7, 147.3);
        chooChooMap.put(69.6, 147.6);
        chooChooMap.put(69.5, 147.9);
        chooChooMap.put(69.4, 148.3);
        chooChooMap.put(69.3, 148.6);
        chooChooMap.put(69.2, 148.9);
        chooChooMap.put(69.1, 149.2);
        chooChooMap.put(69.0, 149.6);
        chooChooMap.put(68.9, 149.9);
        chooChooMap.put(68.8, 150.2);
        chooChooMap.put(68.7, 150.5);
        chooChooMap.put(68.6, 150.9);
        chooChooMap.put(68.5, 151.2);
        chooChooMap.put(68.4, 151.5);
        chooChooMap.put(68.3, 151.8);
        chooChooMap.put(68.2, 152.2);
        chooChooMap.put(68.1, 152.5);
        chooChooMap.put(68.0, 152.8);
        chooChooMap.put(67.9, 153.1);
        chooChooMap.put(67.8, 153.5);
        chooChooMap.put(67.7, 153.8);
        chooChooMap.put(67.6, 154.1);
        chooChooMap.put(67.5, 154.5);
        chooChooMap.put(67.4, 154.8);
        chooChooMap.put(67.3, 155.1);
        chooChooMap.put(67.2, 155.4);
        chooChooMap.put(67.1, 155.8);
        chooChooMap.put(67.0, 156.1);
        chooChooMap.put(66.9, 156.4);
        chooChooMap.put(66.8, 156.8);
        chooChooMap.put(66.7, 157.1);
        chooChooMap.put(66.6, 157.4);
        chooChooMap.put(66.5, 157.8);
        chooChooMap.put(66.4, 158.1);
        chooChooMap.put(66.3, 158.4);
        chooChooMap.put(66.2, 158.7);
        chooChooMap.put(66.1, 159.1);
        chooChooMap.put(66.0, 159.4);
        chooChooMap.put(65.9, 159.7);
        chooChooMap.put(65.8, 160.1);
        chooChooMap.put(65.7, 160.4);
        chooChooMap.put(65.6, 160.7);
        chooChooMap.put(65.5, 161.1);
        chooChooMap.put(65.4, 161.4);
        chooChooMap.put(65.3, 161.8);
        chooChooMap.put(65.2, 162.1);
        chooChooMap.put(65.1, 162.4);
        chooChooMap.put(65.0, 162.8);
        chooChooMap.put(64.9, 163.1);
        chooChooMap.put(64.8, 163.4);
        chooChooMap.put(64.7, 163.8);
        chooChooMap.put(64.6, 164.1);
        chooChooMap.put(64.5, 164.5);
        chooChooMap.put(64.4, 164.8);
        chooChooMap.put(64.3, 165.1);
        chooChooMap.put(64.2, 165.5);
        chooChooMap.put(64.1, 165.8);
        chooChooMap.put(64.0, 166.2);
        chooChooMap.put(63.9, 166.5);
        chooChooMap.put(63.8, 166.8);
        chooChooMap.put(63.7, 167.2);
        chooChooMap.put(63.6, 167.5);
        chooChooMap.put(63.5, 167.9);
        chooChooMap.put(63.4, 168.2);
        chooChooMap.put(63.3, 168.6);
        chooChooMap.put(63.2, 168.9);
        chooChooMap.put(63.1, 169.2);
        chooChooMap.put(63.0, 169.6);
        chooChooMap.put(62.9, 169.9);
        chooChooMap.put(62.8, 170.3);
        chooChooMap.put(62.7, 170.6);
        chooChooMap.put(62.6, 171.0);
        chooChooMap.put(62.5, 171.3);
        chooChooMap.put(62.4, 171.7);
        chooChooMap.put(62.3, 172.0);
        chooChooMap.put(62.2, 172.4);
        chooChooMap.put(62.1, 172.7);
        chooChooMap.put(62.0, 173.1);
        chooChooMap.put(61.9, 173.4);
        chooChooMap.put(61.8, 173.8);
        chooChooMap.put(61.7, 174.1);
        chooChooMap.put(61.6, 174.5);
        chooChooMap.put(61.5, 174.8);
        chooChooMap.put(61.4, 175.2);
        chooChooMap.put(61.3, 175.6);
        chooChooMap.put(61.2, 175.9);
        chooChooMap.put(61.1, 176.3);
        chooChooMap.put(61.0, 176.6);
        chooChooMap.put(60.9, 177.0);
        chooChooMap.put(60.8, 177.3);
        chooChooMap.put(60.7, 177.7);
        chooChooMap.put(60.6, 178.1);
        chooChooMap.put(60.5, 178.4);
        chooChooMap.put(60.4, 178.8);
        chooChooMap.put(60.3, 179.2);
        chooChooMap.put(60.2, 179.5);
        chooChooMap.put(60.1, 179.9);
        chooChooMap.put(60.0, 180.3);
        chooChooMap.put(59.9, 180.6);
        chooChooMap.put(59.8, 181.0);
        chooChooMap.put(59.7, 181.4);
        chooChooMap.put(59.6, 181.7);
        chooChooMap.put(59.5, 182.1);
        chooChooMap.put(59.4, 182.5);
        chooChooMap.put(59.3, 182.8);
        chooChooMap.put(59.2, 183.2);
        chooChooMap.put(59.1, 183.6);
        chooChooMap.put(59.0, 183.9);
        chooChooMap.put(58.9, 184.3);
        chooChooMap.put(58.8, 184.7);
        chooChooMap.put(58.7, 185.1);
        chooChooMap.put(58.6, 185.5);
        chooChooMap.put(58.5, 185.8);
        chooChooMap.put(58.4, 186.2);
        chooChooMap.put(58.3, 186.6);
        chooChooMap.put(58.2, 187.0);
        chooChooMap.put(58.1, 187.4);
        chooChooMap.put(58.0, 187.7);
        chooChooMap.put(57.9, 188.1);
        chooChooMap.put(57.8, 188.5);
        chooChooMap.put(57.7, 188.9);
        chooChooMap.put(57.6, 189.3);
        chooChooMap.put(57.5, 189.7);
        chooChooMap.put(57.4, 190.1);
        chooChooMap.put(57.3, 190.4);
        chooChooMap.put(57.2, 190.8);
        chooChooMap.put(57.1, 191.2);
        chooChooMap.put(57.0, 191.6);
        chooChooMap.put(56.9, 192.0);
        chooChooMap.put(56.8, 192.4);
        chooChooMap.put(56.7, 192.8);
        chooChooMap.put(56.6, 193.2);
        chooChooMap.put(56.5, 193.6);
        chooChooMap.put(56.4, 194.0);
        chooChooMap.put(56.3, 194.4);
        chooChooMap.put(56.2, 194.8);
        chooChooMap.put(56.1, 195.2);
        chooChooMap.put(56.0, 195.6);
        chooChooMap.put(55.9, 196.0);
        chooChooMap.put(55.8, 196.4);
        chooChooMap.put(55.7, 196.9);
        chooChooMap.put(55.6, 197.3);
        chooChooMap.put(55.5, 197.7);
        chooChooMap.put(55.4, 198.1);
        chooChooMap.put(55.3, 198.5);
        chooChooMap.put(55.2, 198.9);
        chooChooMap.put(55.1, 199.4);
        chooChooMap.put(55.0, 199.8);
        chooChooMap.put(54.9, 200.2);
        chooChooMap.put(54.8, 200.6);
        chooChooMap.put(54.7, 201.0);
        chooChooMap.put(54.6, 201.5);
        chooChooMap.put(54.5, 201.9);
        chooChooMap.put(54.4, 202.3);
        chooChooMap.put(54.3, 202.8);
        chooChooMap.put(54.2, 203.2);
        chooChooMap.put(54.1, 203.6);
        chooChooMap.put(54.0, 204.1);
        chooChooMap.put(53.9, 204.5);
        chooChooMap.put(53.8, 204.9);
        chooChooMap.put(53.7, 205.4);
        chooChooMap.put(53.6, 205.8);
        chooChooMap.put(53.5, 206.3);
        chooChooMap.put(53.4, 206.7);
        chooChooMap.put(53.3, 207.2);
        chooChooMap.put(53.2, 207.6);
        chooChooMap.put(53.1, 208.1);
        chooChooMap.put(53.0, 208.5);
        chooChooMap.put(52.9, 209.0);
        chooChooMap.put(52.8, 209.5);
        chooChooMap.put(52.7, 209.9);
        chooChooMap.put(52.6, 210.4);
        chooChooMap.put(52.5, 210.9);
        chooChooMap.put(52.4, 211.3);
        chooChooMap.put(52.3, 211.8);
        chooChooMap.put(52.2, 212.3);
        chooChooMap.put(52.1, 212.8);
        chooChooMap.put(52.0, 213.3);
        chooChooMap.put(51.9, 213.7);
        chooChooMap.put(51.8, 214.2);
        chooChooMap.put(51.7, 214.7);
        chooChooMap.put(51.6, 215.2);
        chooChooMap.put(51.5, 215.7);
        chooChooMap.put(51.4, 216.2);
        chooChooMap.put(51.3, 216.7);
        chooChooMap.put(51.2, 217.2);
        chooChooMap.put(51.1, 217.7);
        chooChooMap.put(51.0, 218.2);
        chooChooMap.put(50.9, 218.7);
        chooChooMap.put(50.8, 219.3);
        chooChooMap.put(50.7, 219.8);
        chooChooMap.put(50.6, 220.3);
        chooChooMap.put(50.5, 220.8);
        chooChooMap.put(50.4, 221.4);
        chooChooMap.put(50.3, 221.9);
        chooChooMap.put(50.2, 222.4);
        chooChooMap.put(50.1, 223.0);
        chooChooMap.put(50.0, 223.5);
        chooChooMap.put(49.9, 224.1);
        chooChooMap.put(49.8, 224.6);
        chooChooMap.put(49.7, 225.2);
        chooChooMap.put(49.6, 225.8);
        chooChooMap.put(49.5, 226.4);
        chooChooMap.put(49.4, 226.9);
        chooChooMap.put(49.3, 227.5);
        chooChooMap.put(49.2, 228.1);
        chooChooMap.put(49.1, 228.7);
        chooChooMap.put(49.0, 229.3);
        chooChooMap.put(48.9, 229.9);
        chooChooMap.put(48.8, 230.5);
        chooChooMap.put(48.7, 231.1);
        chooChooMap.put(48.6, 231.8);
        chooChooMap.put(48.5, 232.4);
        chooChooMap.put(48.4, 233.0);
        chooChooMap.put(48.3, 233.7);
        chooChooMap.put(48.2, 234.3);
        chooChooMap.put(48.1, 235.0);
        chooChooMap.put(48.0, 235.7);
        chooChooMap.put(47.9, 236.4);
        chooChooMap.put(47.8, 237.1);
        chooChooMap.put(47.7, 237.8);
        chooChooMap.put(47.6, 238.5);
        chooChooMap.put(47.5, 239.2);
        chooChooMap.put(47.4, 239.9);
        chooChooMap.put(47.3, 240.7);
        chooChooMap.put(47.2, 241.5);
        chooChooMap.put(47.1, 242.2);
        chooChooMap.put(47.0, 243.0);
        chooChooMap.put(46.9, 243.8);
        chooChooMap.put(46.8, 244.7);
        chooChooMap.put(46.7, 245.5);
        chooChooMap.put(46.6, 246.4);
        chooChooMap.put(46.5, 247.3);
        chooChooMap.put(46.4, 248.2);
        chooChooMap.put(46.3, 249.1);
        chooChooMap.put(46.2, 250.1);
        chooChooMap.put(46.1, 251.1);
        chooChooMap.put(46.0, 252.2);
        chooChooMap.put(45.9, 253.3);
        chooChooMap.put(45.8, 254.4);
        chooChooMap.put(45.7, 255.6);
        chooChooMap.put(45.6, 256.9);
        chooChooMap.put(45.5, 258.2);
        chooChooMap.put(45.4, 259.7);
        chooChooMap.put(45.3, 261.3);
        chooChooMap.put(45.2, 263.1);
        chooChooMap.put(45.1, 265.2);
        chooChooMap.put(45.0, 267.9);
        chooChooMap.put(44.9, 272.9);
        
    }
}