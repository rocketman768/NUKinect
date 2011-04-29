#ifndef _NU_BUFFER_SPEC_
#deifne _NU_BUFFER_SPEC_

class NUBufferSpec {
public:
  static enum BufferMode {
    RGB,
    LUMINANCE
  };

  NUBufferSpec(BufferMode m) { mode = m; }
  BufferMode getMode() const { return mode; }

private:
  BufferMode mode;

};

#endif //  _NU_BUFFER_SPEC_
