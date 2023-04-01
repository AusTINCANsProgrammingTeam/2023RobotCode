package frc.robot.classes;

import java.nio.charset.StandardCharsets;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SerialPortJNI;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort;

public class TimeOfFlightSensor implements AutoCloseable {
  private static class SingleCharSequence implements CharSequence {
    public byte[] data;

    @Override
    public int length() {
      return data.length;
    }

    @Override
    public char charAt(int index) {
      return (char)data[index];
    }

    @Override
    public CharSequence subSequence(int start, int end) {
      return new String(data, start, end, StandardCharsets.UTF_8);
    }

  }

  private static class IntRef {
    int value;
  }

  int parseIntFromIndex(SingleCharSequence charSeq, int readLen, IntRef lastComma) {
    int nextComma = 0;
    try {
      nextComma = findNextComma(charSeq.data, readLen, lastComma.value);
      int value = Integer.parseInt(charSeq, lastComma.value + 1, nextComma, 10);
      lastComma.value = nextComma;
      return value;
    } catch (Exception ex) {
      return 0;
    }
  }

  private int findNextComma(byte[] data, int readLen, int lastComma) {
    while (true) {
      if (readLen <= lastComma + 1 ) {
        return readLen;
      }
      lastComma++;
      if (data[lastComma] == ',') {
        break;
      }
    }
    return lastComma;
  }

  private final AtomicBoolean debugPrints = new AtomicBoolean();
  private boolean hasDistance0;
  private boolean hasDistance1;
  private int distance0;
  private int distance1;
  private double lastReadTime;
  private final ReentrantLock threadLock = new ReentrantLock();
  private final Thread readThread;
  private final AtomicBoolean threadRunning = new AtomicBoolean(true);


  private void threadMain() {
    // Using JNI for a non allocating read
    int port = SerialPortJNI.serialInitializePort((byte)1);
    SerialPortJNI.serialSetBaudRate(port, 115200);
    SerialPortJNI.serialSetDataBits(port, (byte)8);
    SerialPortJNI.serialSetParity(port, (byte)0);
    SerialPortJNI.serialSetStopBits(port, (byte)10);

    SerialPortJNI.serialSetTimeout(port, 1);
    SerialPortJNI.serialEnableTermination(port, '\n');
    
    //int port = SerialPortJNI.serialInitializePort((byte)3);
    HAL.report(tResourceType.kResourceType_SerialPort, 2);

    byte[] buffer = new byte[257];
    SingleCharSequence charSeq = new SingleCharSequence();
    charSeq.data = buffer;
    IntRef lastComma = new IntRef();

    int distance0;
    int distance1;

    while (threadRunning.get()) {
      int read = SerialPortJNI.serialRead(port, buffer, buffer.length - 1);
      if (read <= 0) {
        try {
          threadLock.lock();
          this.hasDistance0 = false;
          this.hasDistance1 = false;
        } finally {
          threadLock.unlock();
        }
        continue;
      }
      if (!threadRunning.get()) {
        break;
      }

      // Trim trailing newline if exists
      if (buffer[read - 1] == '\n') {
        read--;
      }

      if (read == 0) {
        continue;
      }

      lastComma.value = -1;

      distance0 = parseIntFromIndex(charSeq, read, lastComma);
      distance1 = parseIntFromIndex(charSeq, read, lastComma);
      boolean hasDistance0 = distance0 != 0 && distance0 != -1;
      boolean hasDistance1 = distance1 != 0 && distance1 != -1;

      double ts = Timer.getFPGATimestamp();

      try {
        threadLock.lock();
        this.lastReadTime = ts;
        this.hasDistance0 = hasDistance0;
        this.hasDistance1 = hasDistance1;
        if (hasDistance0) {
          this.distance0 = distance0;
        }
        if (hasDistance1) {
          this.distance1 = distance1;
        }
      } finally {
        threadLock.unlock();
      }
    }

    SerialPortJNI.serialClose(port);
  }

  public TimeOfFlightSensor() {
    readThread = new Thread(this::threadMain);
    readThread.setName("TimeOfFlightThread");
    readThread.start();
  }

  public boolean isSensor0Connected() {
    try {
      threadLock.lock();
      return hasDistance0;
    } finally {
      threadLock.unlock();
    }
  }

  public boolean isSensor1Connected() {
    try {
      threadLock.lock();
      return hasDistance1;
    } finally {
      threadLock.unlock();
    }
  }

  public int getDistance0() {
    try {
      threadLock.lock();
      return (hasDistance0) ? distance0 : -1;
    } finally {
      threadLock.unlock();
    }
  }

  public int getDistance1() {
    try {
      threadLock.lock();
      return (hasDistance1) ? distance1 : -1;
    } finally {
      threadLock.unlock();
    }
  }

  public double getLastReadTimestampSeconds() {
    try {
      threadLock.lock();
      return lastReadTime;
    } finally {
      threadLock.unlock();
    }
  }

  void setDebugPrints(boolean debug) {
    debugPrints.set(debug);
  }

  @Override
  public void close() throws Exception {
    threadRunning.set(false);
    readThread.join();
  }
}