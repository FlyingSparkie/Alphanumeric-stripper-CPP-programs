// The sketch is auto-generated with XOD (https://xod.io).
//
// You can compile and upload it to an Arduino-compatible board with
// Arduino IDE.
//
// Rough code overview:
//
// - Configuration section
// - STL shim
// - Immutable list classes and functions
// - XOD runtime environment
// - Native node implementation
// - Program graph definition
//
// Search for comments fenced with '====' and '----' to navigate through
// the major code blocks.

#include <Arduino.h>
#include <inttypes.h>


/*=============================================================================
 *
 *
 * Configuration
 *
 *
 =============================================================================*/

// Uncomment to turn on debug of the program
//#define XOD_DEBUG

// Uncomment to trace the program runtime in the Serial Monitor
//#define XOD_DEBUG_ENABLE_TRACE

/*=============================================================================
 *
 *
 * STL shim. Provides implementation for vital std::* constructs
 *
 *
 =============================================================================*/

namespace xod {
namespace std {

template< class T > struct remove_reference      {typedef T type;};
template< class T > struct remove_reference<T&>  {typedef T type;};
template< class T > struct remove_reference<T&&> {typedef T type;};

template <class T>
typename remove_reference<T>::type&& move(T&& a) {
    return static_cast<typename remove_reference<T>::type&&>(a);
}

} // namespace std
} // namespace xod

/*=============================================================================
 *
 *
 * XOD-specific list/array implementations
 *
 *
 =============================================================================*/

#ifndef XOD_LIST_H
#define XOD_LIST_H

namespace xod {
namespace detail {

/*
 * Cursors are used internaly by iterators and list views. They are not exposed
 * directly to a list consumer.
 *
 * The base `Cursor` is an interface which provides the bare minimum of methods
 * to facilitate a single iteration pass.
 */
template<typename T> class Cursor {
  public:
    virtual ~Cursor() { }
    virtual bool isValid() const = 0;
    virtual bool value(T* out) const = 0;
    virtual void next() = 0;
};

template<typename T> class NilCursor : public Cursor<T> {
  public:
    virtual bool isValid() const { return false; }
    virtual bool value(T*) const { return false; }
    virtual void next() { }
};

} // namespace detail

/*
 * Iterator is an object used to iterate a list once.
 *
 * Users create new iterators by calling `someList.iterate()`.
 * Iterators are created on stack and are supposed to have a
 * short live, e.g. for a duration of `for` loop or node’s
 * `evaluate` function. Iterators can’t be copied.
 *
 * Implemented as a pimpl pattern wrapper over the cursor.
 * Once created for a cursor, an iterator owns that cursor
 * and will delete the cursor object once destroyed itself.
 */
template<typename T>
class Iterator {
  public:
    static Iterator<T> nil() {
        return Iterator<T>(new detail::NilCursor<T>());
    }

    Iterator(detail::Cursor<T>* cursor)
        : _cursor(cursor)
    { }

    ~Iterator() {
        if (_cursor)
            delete _cursor;
    }

    Iterator(const Iterator& that) = delete;
    Iterator& operator=(const Iterator& that) = delete;

    Iterator(Iterator&& it)
        : _cursor(it._cursor)
    {
        it._cursor = nullptr;
    }

    Iterator& operator=(Iterator&& it) {
        auto tmp = it._cursor;
        it._cursor = _cursor;
        _cursor = tmp;
        return *this;
    }

    operator bool() const { return _cursor->isValid(); }

    bool value(T* out) const {
        return _cursor->value(out);
    }

    T operator*() const {
        T out;
        _cursor->value(&out);
        return out;
    }

    Iterator& operator++() {
        _cursor->next();
        return *this;
    }

  private:
    detail::Cursor<T>* _cursor;
};

/*
 * An interface for a list view. A particular list view provides a new
 * kind of iteration over existing data. This way we can use list slices,
 * list concatenations, list rotations, etc without introducing new data
 * buffers. We just change the way already existing data is iterated.
 *
 * ListView is not exposed to a list user directly, it is used internally
 * by the List class. However, deriving a new ListView is necessary if you
 * make a new list/string processing node.
 */
template<typename T> class ListView {
  public:
    virtual Iterator<T> iterate() const = 0;
};

/*
 * The list as it seen by data consumers. Have a single method `iterate`
 * to create a new iterator.
 *
 * Implemented as pimpl pattern wrapper over a list view. Takes pointer
 * to a list view in constructor and expects the view will be alive for
 * the whole life time of the list.
 */
template<typename T> class List {
  public:
    constexpr List()
        : _view(nullptr)
    { }

    List(const ListView<T>* view)
        : _view(view)
    { }

    Iterator<T> iterate() const {
        return _view ? _view->iterate() : Iterator<T>::nil();
    }

    // pre 0.15.0 backward compatibility
    List* operator->() __attribute__ ((deprecated)) { return this; }
    const List* operator->() const __attribute__ ((deprecated)) { return this; }

  private:
    const ListView<T>* _view;
};

/*
 * A list view over an old good plain C array.
 *
 * Expects the array will be alive for the whole life time of the
 * view.
 */
template<typename T> class PlainListView : public ListView<T> {
  public:
    class Cursor : public detail::Cursor<T> {
      public:
        Cursor(const PlainListView* owner)
            : _owner(owner)
            , _idx(0)
        { }

        bool isValid() const override {
            return _idx < _owner->_len;
        }

        bool value(T* out) const override {
            if (!isValid())
                return false;
            *out = _owner->_data[_idx];
            return true;
        }

        void next() override { ++_idx; }

      private:
        const PlainListView* _owner;
        size_t _idx;
    };

  public:
    PlainListView(const T* data, size_t len)
        : _data(data)
        , _len(len)
    { }

    virtual Iterator<T> iterate() const override {
        return Iterator<T>(new Cursor(this));
    }

  private:
    friend class Cursor;
    const T* _data;
    size_t _len;
};

/*
 * A list view over a null-terminated C-String.
 *
 * Expects the char buffer will be alive for the whole life time of the view.
 * You can use string literals as a buffer, since they are persistent for
 * the program execution time.
 */
class CStringView : public ListView<char> {
  public:
    class Cursor : public detail::Cursor<char> {
      public:
        Cursor(const char* str)
            : _ptr(str)
        { }

        bool isValid() const override {
            return (bool)*_ptr;
        }

        bool value(char* out) const override {
            *out = *_ptr;
            return (bool)*_ptr;
        }

        void next() override { ++_ptr; }

      private:
        const char* _ptr;
    };

  public:
    CStringView(const char* str = nullptr)
        : _str(str)
    { }

    CStringView& operator=(const CStringView& rhs) {
        _str = rhs._str;
        return *this;
    }

    virtual Iterator<char> iterate() const override {
        return _str ? Iterator<char>(new Cursor(_str)) : Iterator<char>::nil();
    }

  private:
    friend class Cursor;
    const char* _str;
};

/*
 * A list view over two other lists (Left and Right) which first iterates the
 * left one, and when exhausted, iterates the right one.
 *
 * Expects both Left and Right to be alive for the whole view life time.
 */
template<typename T> class ConcatListView : public ListView<T> {
  public:
    class Cursor : public detail::Cursor<T> {
      public:
        Cursor(Iterator<T>&& left, Iterator<T>&& right)
            : _left(std::move(left))
            , _right(std::move(right))
        { }

        bool isValid() const override {
            return _left || _right;
        }

        bool value(T* out) const override {
            return _left.value(out) || _right.value(out);
        }

        void next() override {
            _left ? ++_left : ++_right;
        }

      private:
        Iterator<T> _left;
        Iterator<T> _right;
    };

  public:
    ConcatListView() { }

    ConcatListView(List<T> left, List<T> right)
        : _left(left)
        , _right(right)
    { }

    ConcatListView& operator=(const ConcatListView& rhs) {
        _left = rhs._left;
        _right = rhs._right;
        return *this;
    }

    virtual Iterator<T> iterate() const override {
        return Iterator<T>(new Cursor(_left.iterate(), _right.iterate()));
    }

  private:
    friend class Cursor;
    List<T> _left;
    List<T> _right;
};

//----------------------------------------------------------------------------
// Text string helpers
//----------------------------------------------------------------------------

using XString = List<char>;

/*
 * List and list view in a single pack. An utility used to define constant
 * string literals in XOD.
 */
class XStringCString : public XString {
  public:
    XStringCString(const char* str)
        : XString(&_view)
        , _view(str)
    { }

  private:
    CStringView _view;
};

} // namespace xod

#endif

/*=============================================================================
 *
 *
 * Functions to work with memory
 *
 *
 =============================================================================*/
#ifndef XOD_NO_PLACEMENT_NEW
// Placement `new` for Arduino
void* operator new(size_t, void* ptr) {
    return ptr;
}
#endif

/*=============================================================================
 *
 *
 * UART Classes, that wraps Serials
 *
 *
 =============================================================================*/

class HardwareSerial;
class SoftwareSerial;

namespace xod {

class Uart {
  private:
    long _baud;

  protected:
    bool _started = false;

  public:
    Uart(long baud) {
        _baud = baud;
    }

    virtual void begin() = 0;

    virtual void end() = 0;

    virtual void flush() = 0;

    virtual bool available() = 0;

    virtual bool writeByte(uint8_t) = 0;

    virtual bool readByte(uint8_t*) = 0;

    virtual SoftwareSerial* toSoftwareSerial() {
      return nullptr;
    }

    virtual HardwareSerial* toHardwareSerial() {
      return nullptr;
    }

    void changeBaudRate(long baud) {
      _baud = baud;
      if (_started) {
        end();
        begin();
      }
    }

    long getBaudRate() const {
      return _baud;
    }

    Stream* toStream() {
      Stream* stream = (Stream*) toHardwareSerial();
      if (stream) return stream;
      return (Stream*) toSoftwareSerial();
    }
};

class HardwareUart : public Uart {
  private:
    HardwareSerial* _serial;

  public:
    HardwareUart(HardwareSerial& hserial, uint32_t baud = 115200) : Uart(baud) {
      _serial = &hserial;
    }

    void begin();
    void end();
    void flush();

    bool available() {
      return (bool) _serial->available();
    }

    bool writeByte(uint8_t byte) {
      return (bool) _serial->write(byte);
    }

    bool readByte(uint8_t* out) {
      int data = _serial->read();
      if (data == -1) return false;
      *out = data;
      return true;
    }

    HardwareSerial* toHardwareSerial() {
      return _serial;
    }
};

void HardwareUart::begin() {
  _started = true;
  _serial->begin(getBaudRate());
};
void HardwareUart::end() {
  _started = false;
  _serial->end();
};
void HardwareUart::flush() {
  _serial->flush();
};

} // namespace xod

/*=============================================================================
 *
 *
 * Basic algorithms for XOD lists
 *
 *
 =============================================================================*/

#ifndef XOD_LIST_FUNCS_H
#define XOD_LIST_FUNCS_H



namespace xod {

/*
 * Folds a list from left. Also known as "reduce".
 */
template<typename T, typename TR>
TR foldl(List<T> xs, TR (*func)(TR, T), TR acc) {
    for (auto it = xs.iterate(); it; ++it)
        acc = func(acc, *it);
    return acc;
}

template<typename T> size_t lengthReducer(size_t len, T) {
    return len + 1;
}

/*
 * Computes length of a list.
 */
template<typename T> size_t length(List<T> xs) {
    return foldl(xs, lengthReducer<T>, (size_t)0);
}

template<typename T> T* dumpReducer(T* buff, T x) {
    *buff = x;
    return buff + 1;
}

/*
 * Copies a list content into a memory buffer.
 *
 * It is expected that `outBuff` has enough size to fit all the data.
 */
template<typename T> size_t dump(List<T> xs, T* outBuff) {
    T* buffEnd = foldl(xs, dumpReducer, outBuff);
    return buffEnd - outBuff;
}

/*
 * Compares two lists.
 */
template<typename T> bool equal(List<T> lhs, List<T> rhs) {
    auto lhsIt = lhs.iterate();
    auto rhsIt = rhs.iterate();

    for (; lhsIt && rhsIt; ++lhsIt, ++rhsIt) {
        if (*lhsIt != *rhsIt) return false;
    }

    return !lhsIt && !rhsIt;
}

} // namespace xod

#endif


/*=============================================================================
 *
 *
 * Runtime
 *
 *
 =============================================================================*/

//----------------------------------------------------------------------------
// Debug routines
//----------------------------------------------------------------------------
#ifndef DEBUG_SERIAL
#  define DEBUG_SERIAL Serial
#endif

#if defined(XOD_DEBUG) && defined(XOD_DEBUG_ENABLE_TRACE)
#  define XOD_TRACE(x)      { DEBUG_SERIAL.print(x); DEBUG_SERIAL.flush(); }
#  define XOD_TRACE_LN(x)   { DEBUG_SERIAL.println(x); DEBUG_SERIAL.flush(); }
#  define XOD_TRACE_F(x)    XOD_TRACE(F(x))
#  define XOD_TRACE_FLN(x)  XOD_TRACE_LN(F(x))
#else
#  define XOD_TRACE(x)
#  define XOD_TRACE_LN(x)
#  define XOD_TRACE_F(x)
#  define XOD_TRACE_FLN(x)
#endif

//----------------------------------------------------------------------------
// PGM space utilities
//----------------------------------------------------------------------------
#define pgm_read_nodeid(address) (pgm_read_word(address))

/*
 * Workaround for bugs:
 * https://github.com/arduino/ArduinoCore-sam/pull/43
 * https://github.com/arduino/ArduinoCore-samd/pull/253
 * Remove after the PRs merge
 */
#if !defined(ARDUINO_ARCH_AVR) && defined(pgm_read_ptr)
#  undef pgm_read_ptr
#  define pgm_read_ptr(addr) (*(const void **)(addr))
#endif

//----------------------------------------------------------------------------
// Compatibilities
//----------------------------------------------------------------------------

#if !defined(ARDUINO_ARCH_AVR)
/*
 * Provide dtostrf function for non-AVR platforms. Although many platforms
 * provide a stub many others do not. And the stub is based on `sprintf`
 * which doesn’t work with floating point formatters on some platforms
 * (e.g. Arduino M0).
 *
 * This is an implementation based on `fcvt` standard function. Taken here:
 * https://forum.arduino.cc/index.php?topic=368720.msg2542614#msg2542614
 */
char *dtostrf(double val, int width, unsigned int prec, char *sout) {
    int decpt, sign, reqd, pad;
    const char *s, *e;
    char *p;
    s = fcvt(val, prec, &decpt, &sign);
    if (prec == 0 && decpt == 0) {
        s = (*s < '5') ? "0" : "1";
        reqd = 1;
    } else {
        reqd = strlen(s);
        if (reqd > decpt) reqd++;
        if (decpt == 0) reqd++;
    }
    if (sign) reqd++;
    p = sout;
    e = p + reqd;
    pad = width - reqd;
    if (pad > 0) {
        e += pad;
        while (pad-- > 0) *p++ = ' ';
    }
    if (sign) *p++ = '-';
    if (decpt <= 0 && prec > 0) {
        *p++ = '0';
        *p++ = '.';
        e++;
        while ( decpt < 0 ) {
            decpt++;
            *p++ = '0';
        }
    }
    while (p < e) {
        *p++ = *s++;
        if (p == e) break;
        if (--decpt == 0) *p++ = '.';
    }
    if (width < 0) {
        pad = (reqd + width) * -1;
        while (pad-- > 0) *p++ = ' ';
    }
    *p = 0;
    return sout;
}
#endif


namespace xod {
//----------------------------------------------------------------------------
// Type definitions
//----------------------------------------------------------------------------
#if __SIZEOF_FLOAT__ == 4
typedef float Number;
#else
typedef double Number;
#endif
typedef bool Logic;
typedef unsigned long TimeMs;
typedef uint8_t DirtyFlags;

//----------------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------------

TimeMs g_transactionTime;
bool g_isSettingUp;

//----------------------------------------------------------------------------
// Metaprogramming utilities
//----------------------------------------------------------------------------

template<typename T> struct always_false {
    enum { value = 0 };
};

//----------------------------------------------------------------------------
// Forward declarations
//----------------------------------------------------------------------------

TimeMs transactionTime();
void runTransaction();

//----------------------------------------------------------------------------
// Engine (private API)
//----------------------------------------------------------------------------

namespace detail {

template<typename NodeT>
bool isTimedOut(const NodeT* node) {
    TimeMs t = node->timeoutAt;
    // TODO: deal with uint32 overflow
    return t && t < transactionTime();
}

// Marks timed out node dirty. Do not reset timeoutAt here to give
// a chance for a node to get a reasonable result from `isTimedOut`
// later during its `evaluate`
template<typename NodeT>
void checkTriggerTimeout(NodeT* node) {
    node->isNodeDirty |= isTimedOut(node);
}

template<typename NodeT>
void clearTimeout(NodeT* node) {
    node->timeoutAt = 0;
}

template<typename NodeT>
void clearStaleTimeout(NodeT* node) {
    if (isTimedOut(node))
        clearTimeout(node);
}

} // namespace detail

//----------------------------------------------------------------------------
// Public API (can be used by native nodes’ `evaluate` functions)
//----------------------------------------------------------------------------

TimeMs transactionTime() {
    return g_transactionTime;
}

bool isSettingUp() {
    return g_isSettingUp;
}

template<typename ContextT>
void setTimeout(ContextT* ctx, TimeMs timeout) {
    ctx->_node->timeoutAt = transactionTime() + timeout;
}

template<typename ContextT>
void clearTimeout(ContextT* ctx) {
    detail::clearTimeout(ctx->_node);
}

template<typename ContextT>
bool isTimedOut(const ContextT* ctx) {
    return detail::isTimedOut(ctx->_node);
}

} // namespace xod

//----------------------------------------------------------------------------
// Entry point
//----------------------------------------------------------------------------
void setup() {
    // FIXME: looks like there is a rounding bug. Waiting for 100ms fights it
    delay(100);
#ifdef XOD_DEBUG
    DEBUG_SERIAL.begin(115200);
#endif
    XOD_TRACE_FLN("\n\nProgram started");

    xod::g_isSettingUp = true;
    xod::runTransaction();
    xod::g_isSettingUp = false;
}

void loop() {
    xod::runTransaction();
}

/*=============================================================================
 *
 *
 * Native node implementations
 *
 *
 =============================================================================*/

namespace xod {

//-----------------------------------------------------------------------------
// xod/core/boot implementation
//-----------------------------------------------------------------------------
namespace xod__core__boot {

struct State {
};

struct Node {
    State state;
    Logic output_BOOT;

    union {
        struct {
            bool isOutputDirty_BOOT : 1;
            bool isNodeDirty : 1;
        };

        DirtyFlags dirtyFlags;
    };
};

struct output_BOOT { };

template<typename PinT> struct ValueType { using T = void; };
template<> struct ValueType<output_BOOT> { using T = Logic; };

struct ContextObject {
    Node* _node;

};

using Context = ContextObject*;

template<typename PinT> typename ValueType<PinT>::T getValue(Context ctx) {
    static_assert(always_false<PinT>::value,
            "Invalid pin descriptor. Expected one of:" \
            "" \
            " output_BOOT");
}

template<> Logic getValue<output_BOOT>(Context ctx) {
    return ctx->_node->output_BOOT;
}

template<typename InputT> bool isInputDirty(Context ctx) {
    static_assert(always_false<InputT>::value,
            "Invalid input descriptor. Expected one of:" \
            "");
    return false;
}

template<typename OutputT> void emitValue(Context ctx, typename ValueType<OutputT>::T val) {
    static_assert(always_false<OutputT>::value,
            "Invalid output descriptor. Expected one of:" \
            " output_BOOT");
}

template<> void emitValue<output_BOOT>(Context ctx, Logic val) {
    ctx->_node->output_BOOT = val;
    ctx->_node->isOutputDirty_BOOT = true;
}

State* getState(Context ctx) {
    return &ctx->_node->state;
}

void evaluate(Context ctx) {
    emitValue<output_BOOT>(ctx, 1);
}

} // namespace xod__core__boot

//-----------------------------------------------------------------------------
// xod/core/any implementation
//-----------------------------------------------------------------------------
namespace xod__core__any {

struct State {
};

struct Node {
    State state;
    Logic output_OUT;

    union {
        struct {
            bool isOutputDirty_OUT : 1;
            bool isNodeDirty : 1;
        };

        DirtyFlags dirtyFlags;
    };
};

struct input_IN1 { };
struct input_IN2 { };
struct output_OUT { };

template<typename PinT> struct ValueType { using T = void; };
template<> struct ValueType<input_IN1> { using T = Logic; };
template<> struct ValueType<input_IN2> { using T = Logic; };
template<> struct ValueType<output_OUT> { using T = Logic; };

struct ContextObject {
    Node* _node;

    Logic _input_IN1;
    Logic _input_IN2;

    bool _isInputDirty_IN1;
    bool _isInputDirty_IN2;
};

using Context = ContextObject*;

template<typename PinT> typename ValueType<PinT>::T getValue(Context ctx) {
    static_assert(always_false<PinT>::value,
            "Invalid pin descriptor. Expected one of:" \
            " input_IN1 input_IN2" \
            " output_OUT");
}

template<> Logic getValue<input_IN1>(Context ctx) {
    return ctx->_input_IN1;
}
template<> Logic getValue<input_IN2>(Context ctx) {
    return ctx->_input_IN2;
}
template<> Logic getValue<output_OUT>(Context ctx) {
    return ctx->_node->output_OUT;
}

template<typename InputT> bool isInputDirty(Context ctx) {
    static_assert(always_false<InputT>::value,
            "Invalid input descriptor. Expected one of:" \
            " input_IN1 input_IN2");
    return false;
}

template<> bool isInputDirty<input_IN1>(Context ctx) {
    return ctx->_isInputDirty_IN1;
}
template<> bool isInputDirty<input_IN2>(Context ctx) {
    return ctx->_isInputDirty_IN2;
}

template<typename OutputT> void emitValue(Context ctx, typename ValueType<OutputT>::T val) {
    static_assert(always_false<OutputT>::value,
            "Invalid output descriptor. Expected one of:" \
            " output_OUT");
}

template<> void emitValue<output_OUT>(Context ctx, Logic val) {
    ctx->_node->output_OUT = val;
    ctx->_node->isOutputDirty_OUT = true;
}

State* getState(Context ctx) {
    return &ctx->_node->state;
}

void evaluate(Context ctx) {
    bool p1 = isInputDirty<input_IN1>(ctx);
    bool p2 = isInputDirty<input_IN2>(ctx);
    if (p1 || p2)
        emitValue<output_OUT>(ctx, true);
}

} // namespace xod__core__any

//-----------------------------------------------------------------------------
// xod/core/delay implementation
//-----------------------------------------------------------------------------
namespace xod__core__delay {

struct State {
};

struct Node {
    State state;
    TimeMs timeoutAt;
    Logic output_DONE;
    Logic output_ACT;

    union {
        struct {
            bool isOutputDirty_DONE : 1;
            bool isOutputDirty_ACT : 1;
            bool isNodeDirty : 1;
        };

        DirtyFlags dirtyFlags;
    };
};

struct input_T { };
struct input_SET { };
struct input_RST { };
struct output_DONE { };
struct output_ACT { };

template<typename PinT> struct ValueType { using T = void; };
template<> struct ValueType<input_T> { using T = Number; };
template<> struct ValueType<input_SET> { using T = Logic; };
template<> struct ValueType<input_RST> { using T = Logic; };
template<> struct ValueType<output_DONE> { using T = Logic; };
template<> struct ValueType<output_ACT> { using T = Logic; };

struct ContextObject {
    Node* _node;

    Number _input_T;
    Logic _input_SET;
    Logic _input_RST;

    bool _isInputDirty_SET;
    bool _isInputDirty_RST;
};

using Context = ContextObject*;

template<typename PinT> typename ValueType<PinT>::T getValue(Context ctx) {
    static_assert(always_false<PinT>::value,
            "Invalid pin descriptor. Expected one of:" \
            " input_T input_SET input_RST" \
            " output_DONE output_ACT");
}

template<> Number getValue<input_T>(Context ctx) {
    return ctx->_input_T;
}
template<> Logic getValue<input_SET>(Context ctx) {
    return ctx->_input_SET;
}
template<> Logic getValue<input_RST>(Context ctx) {
    return ctx->_input_RST;
}
template<> Logic getValue<output_DONE>(Context ctx) {
    return ctx->_node->output_DONE;
}
template<> Logic getValue<output_ACT>(Context ctx) {
    return ctx->_node->output_ACT;
}

template<typename InputT> bool isInputDirty(Context ctx) {
    static_assert(always_false<InputT>::value,
            "Invalid input descriptor. Expected one of:" \
            " input_SET input_RST");
    return false;
}

template<> bool isInputDirty<input_SET>(Context ctx) {
    return ctx->_isInputDirty_SET;
}
template<> bool isInputDirty<input_RST>(Context ctx) {
    return ctx->_isInputDirty_RST;
}

template<typename OutputT> void emitValue(Context ctx, typename ValueType<OutputT>::T val) {
    static_assert(always_false<OutputT>::value,
            "Invalid output descriptor. Expected one of:" \
            " output_DONE output_ACT");
}

template<> void emitValue<output_DONE>(Context ctx, Logic val) {
    ctx->_node->output_DONE = val;
    ctx->_node->isOutputDirty_DONE = true;
}
template<> void emitValue<output_ACT>(Context ctx, Logic val) {
    ctx->_node->output_ACT = val;
    ctx->_node->isOutputDirty_ACT = true;
}

State* getState(Context ctx) {
    return &ctx->_node->state;
}

void evaluate(Context ctx) {
    if (isInputDirty<input_RST>(ctx)) {
        clearTimeout(ctx);
        emitValue<output_ACT>(ctx, false);
    } else if (isInputDirty<input_SET>(ctx)) {
        TimeMs dt = getValue<input_T>(ctx) * 1000;
        setTimeout(ctx, dt);
        emitValue<output_ACT>(ctx, true);
    } else if (isTimedOut(ctx)) {
        emitValue<output_DONE>(ctx, true);
        emitValue<output_ACT>(ctx, false);
    }
}

} // namespace xod__core__delay

//-----------------------------------------------------------------------------
// xod/core/gate(number) implementation
//-----------------------------------------------------------------------------
namespace xod__core__gate__number {

struct State {
};

struct Node {
    State state;
    Number output_OUT;

    union {
        struct {
            bool isOutputDirty_OUT : 1;
            bool isNodeDirty : 1;
        };

        DirtyFlags dirtyFlags;
    };
};

struct input_IN { };
struct input_EN { };
struct output_OUT { };

template<typename PinT> struct ValueType { using T = void; };
template<> struct ValueType<input_IN> { using T = Number; };
template<> struct ValueType<input_EN> { using T = Logic; };
template<> struct ValueType<output_OUT> { using T = Number; };

struct ContextObject {
    Node* _node;

    Number _input_IN;
    Logic _input_EN;

};

using Context = ContextObject*;

template<typename PinT> typename ValueType<PinT>::T getValue(Context ctx) {
    static_assert(always_false<PinT>::value,
            "Invalid pin descriptor. Expected one of:" \
            " input_IN input_EN" \
            " output_OUT");
}

template<> Number getValue<input_IN>(Context ctx) {
    return ctx->_input_IN;
}
template<> Logic getValue<input_EN>(Context ctx) {
    return ctx->_input_EN;
}
template<> Number getValue<output_OUT>(Context ctx) {
    return ctx->_node->output_OUT;
}

template<typename InputT> bool isInputDirty(Context ctx) {
    static_assert(always_false<InputT>::value,
            "Invalid input descriptor. Expected one of:" \
            "");
    return false;
}

template<typename OutputT> void emitValue(Context ctx, typename ValueType<OutputT>::T val) {
    static_assert(always_false<OutputT>::value,
            "Invalid output descriptor. Expected one of:" \
            " output_OUT");
}

template<> void emitValue<output_OUT>(Context ctx, Number val) {
    ctx->_node->output_OUT = val;
    ctx->_node->isOutputDirty_OUT = true;
}

State* getState(Context ctx) {
    return &ctx->_node->state;
}

void evaluate(Context ctx) {
    if (getValue<input_EN>(ctx))
        emitValue<output_OUT>(ctx, getValue<input_IN>(ctx));

}

} // namespace xod__core__gate__number

//-----------------------------------------------------------------------------
// xod/core/cast-to-number(boolean) implementation
//-----------------------------------------------------------------------------
namespace xod__core__cast_to_number__boolean {

//#pragma XOD dirtieness disable

struct State {
};

struct Node {
    State state;
    Number output_OUT;

    union {
        struct {
            bool isNodeDirty : 1;
        };

        DirtyFlags dirtyFlags;
    };
};

struct input_IN { };
struct output_OUT { };

template<typename PinT> struct ValueType { using T = void; };
template<> struct ValueType<input_IN> { using T = Logic; };
template<> struct ValueType<output_OUT> { using T = Number; };

struct ContextObject {
    Node* _node;

    Logic _input_IN;

};

using Context = ContextObject*;

template<typename PinT> typename ValueType<PinT>::T getValue(Context ctx) {
    static_assert(always_false<PinT>::value,
            "Invalid pin descriptor. Expected one of:" \
            " input_IN" \
            " output_OUT");
}

template<> Logic getValue<input_IN>(Context ctx) {
    return ctx->_input_IN;
}
template<> Number getValue<output_OUT>(Context ctx) {
    return ctx->_node->output_OUT;
}

template<typename InputT> bool isInputDirty(Context ctx) {
    static_assert(always_false<InputT>::value,
            "Invalid input descriptor. Expected one of:" \
            "");
    return false;
}

template<typename OutputT> void emitValue(Context ctx, typename ValueType<OutputT>::T val) {
    static_assert(always_false<OutputT>::value,
            "Invalid output descriptor. Expected one of:" \
            " output_OUT");
}

template<> void emitValue<output_OUT>(Context ctx, Number val) {
    ctx->_node->output_OUT = val;
}

State* getState(Context ctx) {
    return &ctx->_node->state;
}

void evaluate(Context ctx) {
    emitValue<output_OUT>(ctx, getValue<input_IN>(ctx) ? 1.0 : 0.0);
}

} // namespace xod__core__cast_to_number__boolean

//-----------------------------------------------------------------------------
// nkrkv/af-motor/dc-motors implementation
//-----------------------------------------------------------------------------
namespace nkrkv__af_motor__dc_motors {

struct State {
    bool begun;
};

struct Node {
    State state;

    union {
        struct {
            bool isNodeDirty : 1;
        };

        DirtyFlags dirtyFlags;
    };
};

struct input_M1 { };
struct input_M2 { };
struct input_M3 { };
struct input_M4 { };

template<typename PinT> struct ValueType { using T = void; };
template<> struct ValueType<input_M1> { using T = Number; };
template<> struct ValueType<input_M2> { using T = Number; };
template<> struct ValueType<input_M3> { using T = Number; };
template<> struct ValueType<input_M4> { using T = Number; };

struct ContextObject {
    Node* _node;

    Number _input_M1;
    Number _input_M2;
    Number _input_M3;
    Number _input_M4;

};

using Context = ContextObject*;

template<typename PinT> typename ValueType<PinT>::T getValue(Context ctx) {
    static_assert(always_false<PinT>::value,
            "Invalid pin descriptor. Expected one of:" \
            " input_M1 input_M2 input_M3 input_M4" \
            "");
}

template<> Number getValue<input_M1>(Context ctx) {
    return ctx->_input_M1;
}
template<> Number getValue<input_M2>(Context ctx) {
    return ctx->_input_M2;
}
template<> Number getValue<input_M3>(Context ctx) {
    return ctx->_input_M3;
}
template<> Number getValue<input_M4>(Context ctx) {
    return ctx->_input_M4;
}

template<typename InputT> bool isInputDirty(Context ctx) {
    static_assert(always_false<InputT>::value,
            "Invalid input descriptor. Expected one of:" \
            "");
    return false;
}

template<typename OutputT> void emitValue(Context ctx, typename ValueType<OutputT>::T val) {
    static_assert(always_false<OutputT>::value,
            "Invalid output descriptor. Expected one of:" \
            "");
}

State* getState(Context ctx) {
    return &ctx->_node->state;
}

// Derive pinout from schematic
// https://cdn-learn.adafruit.com/assets/assets/000/009/769/original/mshieldv1-schem.png

constexpr uint8_t DIR_LATCH = 12;
constexpr uint8_t DIR_SER = 8;
constexpr uint8_t DIR_CLK = 4;
constexpr uint8_t DIR_EN = 7;

constexpr uint8_t M1_PWM = 11; // PMW2A
constexpr uint8_t M1_A = 1 << 2;
constexpr uint8_t M1_B = 1 << 3;

constexpr uint8_t M2_PWM = 3; // PMW2B
constexpr uint8_t M2_A = 1 << 1;
constexpr uint8_t M2_B = 1 << 4;

constexpr uint8_t M3_PWM = 5; // PMW0B
constexpr uint8_t M3_A = 1 << 0;
constexpr uint8_t M3_B = 1 << 6;

constexpr uint8_t M4_PWM = 6; // PMW0A
constexpr uint8_t M4_A = 1 << 5;
constexpr uint8_t M4_B = 1 << 7;

void evaluate(Context ctx) {
    State* state = getState(ctx);
    if (!state->begun) {
        pinMode(DIR_LATCH, OUTPUT);
        pinMode(DIR_SER, OUTPUT);
        pinMode(DIR_CLK, OUTPUT);
        pinMode(DIR_EN, OUTPUT);
        pinMode(M1_PWM, OUTPUT);
        pinMode(M1_A, OUTPUT);
        pinMode(M1_B, OUTPUT);
        pinMode(M2_PWM, OUTPUT);
        pinMode(M2_A, OUTPUT);
        pinMode(M2_B, OUTPUT);
        pinMode(M3_PWM, OUTPUT);
        pinMode(M3_A, OUTPUT);
        pinMode(M3_B, OUTPUT);
        pinMode(M4_PWM, OUTPUT);
        pinMode(M4_A, OUTPUT);
        pinMode(M4_B, OUTPUT);

        state->begun = true;
    }

    Number m1Speed = getValue<input_M1>(ctx);
    Number m2Speed = getValue<input_M2>(ctx);
    Number m3Speed = getValue<input_M3>(ctx);
    Number m4Speed = getValue<input_M4>(ctx);

    // Send direction data to L293s through 74HC595
    uint8_t dirByte =
        (m1Speed < 0 ? M1_A : M1_B) |
        (m2Speed < 0 ? M2_A : M2_B) |
        (m3Speed < 0 ? M3_A : M3_B) |
        (m4Speed < 0 ? M4_A : M4_B);

    digitalWrite(DIR_LATCH, LOW);
    shiftOut(DIR_SER, DIR_CLK, MSBFIRST, dirByte);
    digitalWrite(DIR_LATCH, HIGH);

    // Adjust EN’s of L293s
    analogWrite(M1_PWM, abs(m1Speed) * 255);
    analogWrite(M2_PWM, abs(m2Speed) * 255);
    analogWrite(M3_PWM, abs(m3Speed) * 255);
    analogWrite(M4_PWM, abs(m4Speed) * 255);
}

} // namespace nkrkv__af_motor__dc_motors

//-----------------------------------------------------------------------------
// xod/core/cube implementation
//-----------------------------------------------------------------------------
namespace xod__core__cube {

//#pragma XOD dirtieness disable

struct State {
};

struct Node {
    State state;
    Number output_OUT;

    union {
        struct {
            bool isNodeDirty : 1;
        };

        DirtyFlags dirtyFlags;
    };
};

struct input_IN { };
struct output_OUT { };

template<typename PinT> struct ValueType { using T = void; };
template<> struct ValueType<input_IN> { using T = Number; };
template<> struct ValueType<output_OUT> { using T = Number; };

struct ContextObject {
    Node* _node;

    Number _input_IN;

};

using Context = ContextObject*;

template<typename PinT> typename ValueType<PinT>::T getValue(Context ctx) {
    static_assert(always_false<PinT>::value,
            "Invalid pin descriptor. Expected one of:" \
            " input_IN" \
            " output_OUT");
}

template<> Number getValue<input_IN>(Context ctx) {
    return ctx->_input_IN;
}
template<> Number getValue<output_OUT>(Context ctx) {
    return ctx->_node->output_OUT;
}

template<typename InputT> bool isInputDirty(Context ctx) {
    static_assert(always_false<InputT>::value,
            "Invalid input descriptor. Expected one of:" \
            "");
    return false;
}

template<typename OutputT> void emitValue(Context ctx, typename ValueType<OutputT>::T val) {
    static_assert(always_false<OutputT>::value,
            "Invalid output descriptor. Expected one of:" \
            " output_OUT");
}

template<> void emitValue<output_OUT>(Context ctx, Number val) {
    ctx->_node->output_OUT = val;
}

State* getState(Context ctx) {
    return &ctx->_node->state;
}

void evaluate(Context ctx) {
    Number x = getValue<input_IN>(ctx);
    emitValue<output_OUT>(ctx, x * x * x);
}

} // namespace xod__core__cube

//-----------------------------------------------------------------------------
// xod/core/clock implementation
//-----------------------------------------------------------------------------
namespace xod__core__clock {

struct State {
  TimeMs nextTrig;
};

struct Node {
    State state;
    TimeMs timeoutAt;
    Logic output_TICK;

    union {
        struct {
            bool isOutputDirty_TICK : 1;
            bool isNodeDirty : 1;
        };

        DirtyFlags dirtyFlags;
    };
};

struct input_EN { };
struct input_IVAL { };
struct input_RST { };
struct output_TICK { };

template<typename PinT> struct ValueType { using T = void; };
template<> struct ValueType<input_EN> { using T = Logic; };
template<> struct ValueType<input_IVAL> { using T = Number; };
template<> struct ValueType<input_RST> { using T = Logic; };
template<> struct ValueType<output_TICK> { using T = Logic; };

struct ContextObject {
    Node* _node;

    Logic _input_EN;
    Number _input_IVAL;
    Logic _input_RST;

    bool _isInputDirty_EN;
    bool _isInputDirty_RST;
};

using Context = ContextObject*;

template<typename PinT> typename ValueType<PinT>::T getValue(Context ctx) {
    static_assert(always_false<PinT>::value,
            "Invalid pin descriptor. Expected one of:" \
            " input_EN input_IVAL input_RST" \
            " output_TICK");
}

template<> Logic getValue<input_EN>(Context ctx) {
    return ctx->_input_EN;
}
template<> Number getValue<input_IVAL>(Context ctx) {
    return ctx->_input_IVAL;
}
template<> Logic getValue<input_RST>(Context ctx) {
    return ctx->_input_RST;
}
template<> Logic getValue<output_TICK>(Context ctx) {
    return ctx->_node->output_TICK;
}

template<typename InputT> bool isInputDirty(Context ctx) {
    static_assert(always_false<InputT>::value,
            "Invalid input descriptor. Expected one of:" \
            " input_EN input_RST");
    return false;
}

template<> bool isInputDirty<input_EN>(Context ctx) {
    return ctx->_isInputDirty_EN;
}
template<> bool isInputDirty<input_RST>(Context ctx) {
    return ctx->_isInputDirty_RST;
}

template<typename OutputT> void emitValue(Context ctx, typename ValueType<OutputT>::T val) {
    static_assert(always_false<OutputT>::value,
            "Invalid output descriptor. Expected one of:" \
            " output_TICK");
}

template<> void emitValue<output_TICK>(Context ctx, Logic val) {
    ctx->_node->output_TICK = val;
    ctx->_node->isOutputDirty_TICK = true;
}

State* getState(Context ctx) {
    return &ctx->_node->state;
}

void evaluate(Context ctx) {
    State* state = getState(ctx);
    TimeMs tNow = transactionTime();
    TimeMs dt = getValue<input_IVAL>(ctx) * 1000;
    TimeMs tNext = tNow + dt;

    if (isInputDirty<input_RST>(ctx) || isInputDirty<input_EN>(ctx)) {
        // Handle enable/disable/reset
        if (dt <= 0 || !getValue<input_EN>(ctx)) {
            // Disable timeout loop on zero IVAL or explicit false on EN
            state->nextTrig = 0;
            clearTimeout(ctx);
        } else if (state->nextTrig < tNow || state->nextTrig > tNext) {
            // Start timeout from scratch
            state->nextTrig = tNext;
            setTimeout(ctx, dt);
        }
    }

    if (isTimedOut(ctx)) {
        emitValue<output_TICK>(ctx, 1);
        state->nextTrig = tNext;
        setTimeout(ctx, dt);
    }
}

} // namespace xod__core__clock

//-----------------------------------------------------------------------------
// xod/core/pwm-output implementation
//-----------------------------------------------------------------------------
namespace xod__core__pwm_output {

struct State {
    int configuredPort = -1;
};

struct Node {
    State state;

    union {
        struct {
            bool isNodeDirty : 1;
        };

        DirtyFlags dirtyFlags;
    };
};

struct input_PORT { };
struct input_DUTY { };

template<typename PinT> struct ValueType { using T = void; };
template<> struct ValueType<input_PORT> { using T = Number; };
template<> struct ValueType<input_DUTY> { using T = Number; };

struct ContextObject {
    Node* _node;

    Number _input_PORT;
    Number _input_DUTY;

};

using Context = ContextObject*;

template<typename PinT> typename ValueType<PinT>::T getValue(Context ctx) {
    static_assert(always_false<PinT>::value,
            "Invalid pin descriptor. Expected one of:" \
            " input_PORT input_DUTY" \
            "");
}

template<> Number getValue<input_PORT>(Context ctx) {
    return ctx->_input_PORT;
}
template<> Number getValue<input_DUTY>(Context ctx) {
    return ctx->_input_DUTY;
}

template<typename InputT> bool isInputDirty(Context ctx) {
    static_assert(always_false<InputT>::value,
            "Invalid input descriptor. Expected one of:" \
            "");
    return false;
}

template<typename OutputT> void emitValue(Context ctx, typename ValueType<OutputT>::T val) {
    static_assert(always_false<OutputT>::value,
            "Invalid output descriptor. Expected one of:" \
            "");
}

State* getState(Context ctx) {
    return &ctx->_node->state;
}

void evaluate(Context ctx) {
    State* state = getState(ctx);
    const int port = (int)getValue<input_PORT>(ctx);
    if (port != state->configuredPort) {
        ::pinMode(port, OUTPUT);
        // Store configured port so to avoid repeating `pinMode` call if just
        // SIG is updated
        state->configuredPort = port;
    }

    auto duty = getValue<input_DUTY>(ctx);
    duty = duty > 1 ? 1 : (duty < 0 ? 0 : duty);

    uint8_t val = (uint8_t)(duty * 255.0);
    ::analogWrite(port, val);
}

} // namespace xod__core__pwm_output

//-----------------------------------------------------------------------------
// gweimer/hc-sr04-ultrasonic/ultrasonic-range implementation
//-----------------------------------------------------------------------------
namespace gweimer__hc_sr04_ultrasonic__ultrasonic_range {

struct State {
    TimeMs cooldownUntil = 0;
};

struct Node {
    State state;
    Number output_Dm;

    union {
        struct {
            bool isOutputDirty_Dm : 1;
            bool isNodeDirty : 1;
        };

        DirtyFlags dirtyFlags;
    };
};

struct input_TRIG { };
struct input_ECHO { };
struct input_PING { };
struct output_Dm { };

template<typename PinT> struct ValueType { using T = void; };
template<> struct ValueType<input_TRIG> { using T = Number; };
template<> struct ValueType<input_ECHO> { using T = Number; };
template<> struct ValueType<input_PING> { using T = Logic; };
template<> struct ValueType<output_Dm> { using T = Number; };

struct ContextObject {
    Node* _node;

    Number _input_TRIG;
    Number _input_ECHO;
    Logic _input_PING;

    bool _isInputDirty_PING;
};

using Context = ContextObject*;

template<typename PinT> typename ValueType<PinT>::T getValue(Context ctx) {
    static_assert(always_false<PinT>::value,
            "Invalid pin descriptor. Expected one of:" \
            " input_TRIG input_ECHO input_PING" \
            " output_Dm");
}

template<> Number getValue<input_TRIG>(Context ctx) {
    return ctx->_input_TRIG;
}
template<> Number getValue<input_ECHO>(Context ctx) {
    return ctx->_input_ECHO;
}
template<> Logic getValue<input_PING>(Context ctx) {
    return ctx->_input_PING;
}
template<> Number getValue<output_Dm>(Context ctx) {
    return ctx->_node->output_Dm;
}

template<typename InputT> bool isInputDirty(Context ctx) {
    static_assert(always_false<InputT>::value,
            "Invalid input descriptor. Expected one of:" \
            " input_PING");
    return false;
}

template<> bool isInputDirty<input_PING>(Context ctx) {
    return ctx->_isInputDirty_PING;
}

template<typename OutputT> void emitValue(Context ctx, typename ValueType<OutputT>::T val) {
    static_assert(always_false<OutputT>::value,
            "Invalid output descriptor. Expected one of:" \
            " output_Dm");
}

template<> void emitValue<output_Dm>(Context ctx, Number val) {
    ctx->_node->output_Dm = val;
    ctx->_node->isOutputDirty_Dm = true;
}

State* getState(Context ctx) {
    return &ctx->_node->state;
}

constexpr uint32_t HCSR04_COOLDOWN_MS = 60;
constexpr uint32_t HCSR04_MAX_START_US = 5000;
constexpr uint32_t HCSR04_MAX_ROUNDTRIP_US =
    1000ul * 1000ul         // seconds to μs
    * 4ul                   // max meters
    * 2ul                   // to the moon and back
    / 340ul;                // sound speed

enum Status {
    HCSR04_OK,
    HCSR04_NO_ECHO,
    HCSR04_WRONG_CONNECTION
};

Status pingSync(uint8_t echoPort, uint8_t trigPort, uint32_t* outRoundtripUs) {
    uint32_t maxUs;

    // Request measurement: make a pulse for 10 μs.
    pinMode(trigPort, OUTPUT);
    digitalWrite(trigPort, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPort, LOW);

    // Wait for echo pin rise which means ultrasonic burst success
    maxUs = micros() + HCSR04_MAX_ROUNDTRIP_US;
    pinMode(echoPort, INPUT);
    while (digitalRead(echoPort) == LOW) {
        if (micros() > maxUs) {
            *outRoundtripUs = HCSR04_MAX_ROUNDTRIP_US;
            return HCSR04_OK;
            //return HCSR04_WRONG_CONNECTION;
        }
    }

    // Now wait for echo line to be pulled back low which means echo capture
    uint32_t tStart = micros();
    maxUs = tStart + HCSR04_MAX_ROUNDTRIP_US;
    while (digitalRead(echoPort) == HIGH) {
        if (micros() > maxUs) {
            *outRoundtripUs = -HCSR04_MAX_ROUNDTRIP_US;
            return HCSR04_OK;
            //return HCSR04_NO_ECHO;
        }
    }

    *outRoundtripUs = micros() - tStart;
    return HCSR04_OK;
}

void evaluate(Context ctx) {
    if (!isInputDirty<input_PING>(ctx))
        return;

    auto state = getState(ctx);
    if (millis() < state->cooldownUntil) {
        emitValue<output_Dm>(ctx, -1);
        return;
    }

    auto echoPort = (uint8_t)getValue<input_ECHO>(ctx);
    auto trigPort = (uint8_t)getValue<input_TRIG>(ctx);

    uint32_t t;
    auto status = pingSync(
        (uint8_t)getValue<input_ECHO>(ctx),
        (uint8_t)getValue<input_TRIG>(ctx),
        &t
    );

    // We should not ping too often as the PCB of the sensor could resonate
    state->cooldownUntil = millis() + HCSR04_COOLDOWN_MS;

    if (status == HCSR04_OK)
        emitValue<output_Dm>(ctx, Number(t) / 1000000.0 * 170);
}

} // namespace gweimer__hc_sr04_ultrasonic__ultrasonic_range

//-----------------------------------------------------------------------------
// xod/core/less implementation
//-----------------------------------------------------------------------------
namespace xod__core__less {

//#pragma XOD dirtieness disable

struct State {};

struct Node {
    State state;
    Logic output_OUT;

    union {
        struct {
            bool isNodeDirty : 1;
        };

        DirtyFlags dirtyFlags;
    };
};

struct input_IN1 { };
struct input_IN2 { };
struct output_OUT { };

template<typename PinT> struct ValueType { using T = void; };
template<> struct ValueType<input_IN1> { using T = Number; };
template<> struct ValueType<input_IN2> { using T = Number; };
template<> struct ValueType<output_OUT> { using T = Logic; };

struct ContextObject {
    Node* _node;

    Number _input_IN1;
    Number _input_IN2;

};

using Context = ContextObject*;

template<typename PinT> typename ValueType<PinT>::T getValue(Context ctx) {
    static_assert(always_false<PinT>::value,
            "Invalid pin descriptor. Expected one of:" \
            " input_IN1 input_IN2" \
            " output_OUT");
}

template<> Number getValue<input_IN1>(Context ctx) {
    return ctx->_input_IN1;
}
template<> Number getValue<input_IN2>(Context ctx) {
    return ctx->_input_IN2;
}
template<> Logic getValue<output_OUT>(Context ctx) {
    return ctx->_node->output_OUT;
}

template<typename InputT> bool isInputDirty(Context ctx) {
    static_assert(always_false<InputT>::value,
            "Invalid input descriptor. Expected one of:" \
            "");
    return false;
}

template<typename OutputT> void emitValue(Context ctx, typename ValueType<OutputT>::T val) {
    static_assert(always_false<OutputT>::value,
            "Invalid output descriptor. Expected one of:" \
            " output_OUT");
}

template<> void emitValue<output_OUT>(Context ctx, Logic val) {
    ctx->_node->output_OUT = val;
}

State* getState(Context ctx) {
    return &ctx->_node->state;
}

void evaluate(Context ctx) {
    auto lhs = getValue<input_IN1>(ctx);
    auto rhs = getValue<input_IN2>(ctx);
    emitValue<output_OUT>(ctx, lhs < rhs);
}

} // namespace xod__core__less

//-----------------------------------------------------------------------------
// xod/core/cast-to-pulse(boolean) implementation
//-----------------------------------------------------------------------------
namespace xod__core__cast_to_pulse__boolean {

struct State {
  bool state = false;
};

struct Node {
    State state;
    Logic output_OUT;

    union {
        struct {
            bool isOutputDirty_OUT : 1;
            bool isNodeDirty : 1;
        };

        DirtyFlags dirtyFlags;
    };
};

struct input_IN { };
struct output_OUT { };

template<typename PinT> struct ValueType { using T = void; };
template<> struct ValueType<input_IN> { using T = Logic; };
template<> struct ValueType<output_OUT> { using T = Logic; };

struct ContextObject {
    Node* _node;

    Logic _input_IN;

};

using Context = ContextObject*;

template<typename PinT> typename ValueType<PinT>::T getValue(Context ctx) {
    static_assert(always_false<PinT>::value,
            "Invalid pin descriptor. Expected one of:" \
            " input_IN" \
            " output_OUT");
}

template<> Logic getValue<input_IN>(Context ctx) {
    return ctx->_input_IN;
}
template<> Logic getValue<output_OUT>(Context ctx) {
    return ctx->_node->output_OUT;
}

template<typename InputT> bool isInputDirty(Context ctx) {
    static_assert(always_false<InputT>::value,
            "Invalid input descriptor. Expected one of:" \
            "");
    return false;
}

template<typename OutputT> void emitValue(Context ctx, typename ValueType<OutputT>::T val) {
    static_assert(always_false<OutputT>::value,
            "Invalid output descriptor. Expected one of:" \
            " output_OUT");
}

template<> void emitValue<output_OUT>(Context ctx, Logic val) {
    ctx->_node->output_OUT = val;
    ctx->_node->isOutputDirty_OUT = true;
}

State* getState(Context ctx) {
    return &ctx->_node->state;
}

void evaluate(Context ctx) {
    State* state = getState(ctx);
    auto newValue = getValue<input_IN>(ctx);

    if (newValue == true && state->state == false)
        emitValue<output_OUT>(ctx, 1);

    state->state = newValue;
}

} // namespace xod__core__cast_to_pulse__boolean

//-----------------------------------------------------------------------------
// xod/core/defer(pulse) implementation
//-----------------------------------------------------------------------------
namespace xod__core__defer__pulse {

struct State {
};

struct Node {
    State state;
    TimeMs timeoutAt;
    Logic output_OUT;

    union {
        struct {
            bool isOutputDirty_OUT : 1;
            bool isNodeDirty : 1;
        };

        DirtyFlags dirtyFlags;
    };
};

struct input_IN { };
struct output_OUT { };

template<typename PinT> struct ValueType { using T = void; };
template<> struct ValueType<input_IN> { using T = Logic; };
template<> struct ValueType<output_OUT> { using T = Logic; };

struct ContextObject {
    Node* _node;

    Logic _input_IN;

    bool _isInputDirty_IN;
};

using Context = ContextObject*;

template<typename PinT> typename ValueType<PinT>::T getValue(Context ctx) {
    static_assert(always_false<PinT>::value,
            "Invalid pin descriptor. Expected one of:" \
            " input_IN" \
            " output_OUT");
}

template<> Logic getValue<input_IN>(Context ctx) {
    return ctx->_input_IN;
}
template<> Logic getValue<output_OUT>(Context ctx) {
    return ctx->_node->output_OUT;
}

template<typename InputT> bool isInputDirty(Context ctx) {
    static_assert(always_false<InputT>::value,
            "Invalid input descriptor. Expected one of:" \
            " input_IN");
    return false;
}

template<> bool isInputDirty<input_IN>(Context ctx) {
    return ctx->_isInputDirty_IN;
}

template<typename OutputT> void emitValue(Context ctx, typename ValueType<OutputT>::T val) {
    static_assert(always_false<OutputT>::value,
            "Invalid output descriptor. Expected one of:" \
            " output_OUT");
}

template<> void emitValue<output_OUT>(Context ctx, Logic val) {
    ctx->_node->output_OUT = val;
    ctx->_node->isOutputDirty_OUT = true;
}

State* getState(Context ctx) {
    return &ctx->_node->state;
}

void evaluate(Context ctx) {
    if (isInputDirty<input_IN>(ctx)) { // This happens only when all nodes are evaluated
        setTimeout(ctx, 0);
    } else if (isTimedOut(ctx)) {
        emitValue<output_OUT>(ctx, true);
    }
}

} // namespace xod__core__defer__pulse

} // namespace xod


/*=============================================================================
 *
 *
 * Main loop components
 *
 *
 =============================================================================*/

namespace xod {

// Define/allocate persistent storages (state, timeout, output data) for all nodes
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

constexpr Logic node_0_output_BOOT = false;

constexpr Logic node_1_output_OUT = false;

constexpr Number node_2_output_VAL = 1;

constexpr Number node_3_output_VAL = 0;

constexpr Number node_4_output_VAL = 0;

constexpr Number node_5_output_VAL = 0;

constexpr Number node_6_output_VAL = 1;

constexpr Number node_7_output_VAL = 0;

constexpr Number node_8_output_VAL = 0;

constexpr Number node_9_output_VAL = 0;

constexpr Number node_10_output_VAL = 0;

constexpr Number node_11_output_VAL = 0;

constexpr Number node_12_output_VAL = -1;

constexpr Number node_13_output_VAL = 2;

constexpr Number node_14_output_VAL = 19;

constexpr Number node_15_output_VAL = -1;

constexpr Number node_16_output_VAL = 1;

constexpr Number node_17_output_VAL = 2;

constexpr Number node_18_output_VAL = 0;

constexpr Number node_19_output_VAL = 0;

constexpr Number node_20_output_VAL = 18;

constexpr Number node_21_output_VAL = 19;

constexpr Number node_22_output_VAL = -1;

constexpr Number node_23_output_VAL = 2;

constexpr Number node_24_output_VAL = 0;

constexpr Number node_25_output_VAL = 0;

constexpr Number node_26_output_VAL = 1;

constexpr Number node_27_output_VAL = 18;

constexpr Number node_28_output_VAL = 19;

constexpr Number node_29_output_VAL = 2;

constexpr Number node_30_output_VAL = 1;

constexpr Number node_31_output_VAL = 0;

constexpr Number node_32_output_VAL = 0;

constexpr Number node_33_output_VAL = 16;

constexpr Number node_34_output_VAL = 1;

constexpr Number node_35_output_VAL = 2;

constexpr Number node_36_output_VAL = 0;

constexpr Number node_37_output_VAL = 0;

constexpr Number node_38_output_VAL = 0;

constexpr Number node_39_output_VAL = 16;

constexpr Number node_40_output_VAL = 17;

constexpr Number node_41_output_VAL = 1;

constexpr Number node_42_output_VAL = 0;

constexpr Number node_43_output_VAL = 0;

constexpr Number node_44_output_VAL = 0;

constexpr Number node_45_output_VAL = 2;

constexpr Number node_46_output_VAL = 0;

constexpr Number node_47_output_VAL = 0;

constexpr Number node_48_output_VAL = 0;

constexpr Number node_49_output_VAL = 2;

constexpr Number node_50_output_VAL = 1;

constexpr Number node_51_output_VAL = 0;

constexpr Number node_52_output_VAL = 0;

constexpr Number node_53_output_VAL = 16;

constexpr Number node_54_output_VAL = -1;

constexpr Number node_55_output_VAL = 2;

constexpr Number node_56_output_VAL = 0;

constexpr Number node_57_output_VAL = 0;

constexpr Number node_58_output_VAL = 1;

constexpr Number node_59_output_VAL = 18;

constexpr Number node_60_output_VAL = 19;

constexpr Number node_61_output_VAL = -1;

constexpr Number node_62_output_VAL = 1;

constexpr Number node_63_output_VAL = 2;

constexpr Number node_64_output_VAL = 0;

constexpr Number node_65_output_VAL = 0;

constexpr Number node_66_output_VAL = 18;

constexpr Number node_67_output_VAL = 19;

constexpr Number node_68_output_VAL = 1;

constexpr Number node_69_output_VAL = 2;

constexpr Number node_70_output_VAL = 0;

constexpr Number node_71_output_VAL = 0;

constexpr Number node_72_output_VAL = 0;

constexpr Number node_73_output_VAL = 16;

constexpr Number node_74_output_VAL = 17;

constexpr Number node_75_output_VAL = 2;

constexpr Number node_76_output_VAL = 1;

constexpr Number node_77_output_VAL = 0;

constexpr Number node_78_output_VAL = 0;

constexpr Number node_79_output_VAL = 0;

constexpr Number node_80_output_VAL = 16;

constexpr Number node_81_output_VAL = 17;

constexpr Number node_82_output_VAL = .1;

constexpr Number node_83_output_VAL = .2;

constexpr Number node_84_output_VAL = 14;

constexpr Number node_85_output_VAL = 15;

constexpr Number node_86_output_VAL = .45;

constexpr Logic node_87_output_VAL = true;

constexpr Number node_88_output_VAL = 1;

constexpr Number node_89_output_VAL = 0;

constexpr Number node_90_output_VAL = 0;

constexpr Number node_91_output_VAL = -1;

constexpr Number node_92_output_VAL = 2;

constexpr Number node_93_output_VAL = 19;

constexpr Number node_94_output_VAL = 0;

constexpr Number node_95_output_VAL = 0;

constexpr Number node_96_output_VAL = -1;

constexpr Number node_97_output_VAL = 2;

constexpr Number node_98_output_VAL = 19;

constexpr Number node_99_output_VAL = 2;

constexpr Number node_100_output_VAL = 1;

constexpr Number node_101_output_VAL = 0;

constexpr Number node_102_output_VAL = 0;

constexpr Number node_103_output_VAL = 16;

constexpr Logic node_104_output_OUT = false;

constexpr Logic node_105_output_DONE = false;
constexpr Logic node_105_output_ACT = false;

constexpr Logic node_106_output_DONE = false;
constexpr Logic node_106_output_ACT = false;

constexpr Logic node_107_output_DONE = false;
constexpr Logic node_107_output_ACT = false;

constexpr Logic node_108_output_DONE = false;
constexpr Logic node_108_output_ACT = false;

constexpr Logic node_109_output_DONE = false;
constexpr Logic node_109_output_ACT = false;

constexpr Logic node_110_output_DONE = false;
constexpr Logic node_110_output_ACT = false;

constexpr Logic node_111_output_DONE = false;
constexpr Logic node_111_output_ACT = false;

constexpr Logic node_112_output_DONE = false;
constexpr Logic node_112_output_ACT = false;

constexpr Logic node_113_output_DONE = false;
constexpr Logic node_113_output_ACT = false;

constexpr Logic node_114_output_DONE = false;
constexpr Logic node_114_output_ACT = false;

constexpr Logic node_115_output_DONE = false;
constexpr Logic node_115_output_ACT = false;

constexpr Logic node_116_output_DONE = false;
constexpr Logic node_116_output_ACT = false;

constexpr Logic node_117_output_DONE = false;
constexpr Logic node_117_output_ACT = false;

constexpr Logic node_118_output_DONE = false;
constexpr Logic node_118_output_ACT = false;

constexpr Logic node_119_output_DONE = false;
constexpr Logic node_119_output_ACT = false;

constexpr Logic node_120_output_DONE = false;
constexpr Logic node_120_output_ACT = false;

constexpr Logic node_121_output_DONE = false;
constexpr Logic node_121_output_ACT = false;

constexpr Logic node_122_output_OUT = false;

constexpr Number node_123_output_OUT = 0;

constexpr Number node_124_output_OUT = 0;

constexpr Number node_125_output_OUT = 0;

constexpr Number node_126_output_OUT = 0;

constexpr Number node_127_output_OUT = 0;

constexpr Number node_128_output_OUT = 0;

constexpr Number node_129_output_OUT = 0;

constexpr Number node_130_output_OUT = 0;

constexpr Number node_131_output_OUT = 0;

constexpr Number node_132_output_OUT = 0;

constexpr Number node_133_output_OUT = 0;

constexpr Number node_134_output_OUT = 0;

constexpr Number node_135_output_OUT = 0;

constexpr Number node_136_output_OUT = 0;

constexpr Number node_137_output_OUT = 0;

constexpr Number node_138_output_OUT = 0;

constexpr Number node_139_output_OUT = 0;

constexpr Number node_140_output_OUT = 0;

constexpr Number node_141_output_OUT = 0;

constexpr Number node_142_output_OUT = 0;

constexpr Number node_143_output_OUT = 0;

constexpr Number node_144_output_OUT = 0;

constexpr Number node_145_output_OUT = 0;

constexpr Number node_146_output_OUT = 0;

constexpr Number node_147_output_OUT = 0;

constexpr Number node_148_output_OUT = 0;

constexpr Number node_149_output_OUT = 0;

constexpr Number node_150_output_OUT = 0;

constexpr Number node_151_output_OUT = 0;

constexpr Number node_152_output_OUT = 0;

constexpr Number node_153_output_OUT = 0;

constexpr Number node_154_output_OUT = 0;

constexpr Number node_155_output_OUT = 0;

constexpr Number node_156_output_OUT = 0;

constexpr Number node_157_output_OUT = 0;

constexpr Number node_158_output_OUT = 0;

constexpr Number node_159_output_OUT = 0;

constexpr Number node_160_output_OUT = 0;

constexpr Number node_161_output_OUT = 0;

constexpr Number node_162_output_OUT = 0;

constexpr Number node_163_output_OUT = 0;

constexpr Number node_164_output_OUT = 0;

constexpr Number node_165_output_OUT = 0;

constexpr Number node_166_output_OUT = 0;

constexpr Logic node_167_output_OUT = false;

constexpr Number node_171_output_OUT = 0;

constexpr Number node_173_output_OUT = 0;

constexpr Number node_174_output_OUT = 0;

constexpr Number node_176_output_OUT = 0;

constexpr Number node_177_output_OUT = 0;

constexpr Number node_179_output_OUT = 0;

constexpr Number node_181_output_OUT = 0;

constexpr Number node_182_output_OUT = 0;

constexpr Number node_186_output_OUT = 0;

constexpr Number node_188_output_OUT = 0;

constexpr Number node_189_output_OUT = 0;

constexpr Number node_191_output_OUT = 0;

constexpr Number node_192_output_OUT = 0;

constexpr Number node_194_output_OUT = 0;

constexpr Number node_195_output_OUT = 0;

constexpr Number node_197_output_OUT = 0;

constexpr Number node_198_output_OUT = 0;

constexpr Number node_200_output_OUT = 0;

constexpr Number node_202_output_OUT = 0;

constexpr Number node_204_output_OUT = 0;

constexpr Logic node_205_output_TICK = false;

constexpr Number node_226_output_Dm = 0;

constexpr Logic node_227_output_OUT = false;

constexpr Logic node_228_output_OUT = false;

constexpr Logic node_229_output_OUT = false;

constexpr Logic node_230_output_OUT = false;

constexpr Logic node_231_output_OUT = false;

constexpr Logic node_232_output_OUT = false;

constexpr Logic node_233_output_OUT = false;

constexpr Logic node_234_output_OUT = false;

constexpr Logic node_235_output_OUT = false;

constexpr Logic node_236_output_OUT = false;

constexpr Logic node_237_output_OUT = false;

constexpr Logic node_238_output_OUT = false;

constexpr Logic node_239_output_OUT = false;

constexpr Logic node_240_output_OUT = false;

constexpr Logic node_241_output_OUT = false;

constexpr Logic node_242_output_OUT = false;

constexpr Logic node_243_output_OUT = false;

constexpr Logic node_244_output_OUT = false;

constexpr Logic node_245_output_OUT = false;

constexpr Logic node_246_output_OUT = false;

constexpr Logic node_247_output_OUT = false;

constexpr Logic node_248_output_OUT = false;

constexpr Logic node_249_output_OUT = false;

constexpr Logic node_250_output_OUT = false;

constexpr Logic node_251_output_OUT = false;

constexpr Logic node_252_output_OUT = false;

constexpr Logic node_253_output_OUT = false;

constexpr Logic node_254_output_OUT = false;

constexpr Logic node_255_output_OUT = false;

constexpr Logic node_256_output_OUT = false;

constexpr Logic node_257_output_OUT = false;

constexpr Logic node_258_output_OUT = false;

constexpr Logic node_259_output_OUT = false;

constexpr Logic node_260_output_OUT = false;

#pragma GCC diagnostic pop

xod__core__boot::Node node_0 = {
    xod__core__boot::State(), // state default
    node_0_output_BOOT, // output BOOT default
    false, // BOOT dirty
    true // node itself dirty
};
xod__core__any::Node node_1 = {
    xod__core__any::State(), // state default
    node_1_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__any::Node node_104 = {
    xod__core__any::State(), // state default
    node_104_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__delay::Node node_105 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_105_output_DONE, // output DONE default
    node_105_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__delay::Node node_106 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_106_output_DONE, // output DONE default
    node_106_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__delay::Node node_107 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_107_output_DONE, // output DONE default
    node_107_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__delay::Node node_108 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_108_output_DONE, // output DONE default
    node_108_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__delay::Node node_109 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_109_output_DONE, // output DONE default
    node_109_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__delay::Node node_110 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_110_output_DONE, // output DONE default
    node_110_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__delay::Node node_111 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_111_output_DONE, // output DONE default
    node_111_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__delay::Node node_112 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_112_output_DONE, // output DONE default
    node_112_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__delay::Node node_113 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_113_output_DONE, // output DONE default
    node_113_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__delay::Node node_114 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_114_output_DONE, // output DONE default
    node_114_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__delay::Node node_115 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_115_output_DONE, // output DONE default
    node_115_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__delay::Node node_116 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_116_output_DONE, // output DONE default
    node_116_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__delay::Node node_117 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_117_output_DONE, // output DONE default
    node_117_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__delay::Node node_118 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_118_output_DONE, // output DONE default
    node_118_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__delay::Node node_119 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_119_output_DONE, // output DONE default
    node_119_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__delay::Node node_120 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_120_output_DONE, // output DONE default
    node_120_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__delay::Node node_121 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_121_output_DONE, // output DONE default
    node_121_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__any::Node node_122 = {
    xod__core__any::State(), // state default
    node_122_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__gate__number::Node node_123 = {
    xod__core__gate__number::State(), // state default
    node_123_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__gate__number::Node node_124 = {
    xod__core__gate__number::State(), // state default
    node_124_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__gate__number::Node node_125 = {
    xod__core__gate__number::State(), // state default
    node_125_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_126 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_126_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__gate__number::Node node_127 = {
    xod__core__gate__number::State(), // state default
    node_127_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__gate__number::Node node_128 = {
    xod__core__gate__number::State(), // state default
    node_128_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_129 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_129_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_130 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_130_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__gate__number::Node node_131 = {
    xod__core__gate__number::State(), // state default
    node_131_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__gate__number::Node node_132 = {
    xod__core__gate__number::State(), // state default
    node_132_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_133 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_133_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_134 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_134_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__gate__number::Node node_135 = {
    xod__core__gate__number::State(), // state default
    node_135_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_136 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_136_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__gate__number::Node node_137 = {
    xod__core__gate__number::State(), // state default
    node_137_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__gate__number::Node node_138 = {
    xod__core__gate__number::State(), // state default
    node_138_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_139 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_139_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_140 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_140_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__gate__number::Node node_141 = {
    xod__core__gate__number::State(), // state default
    node_141_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__gate__number::Node node_142 = {
    xod__core__gate__number::State(), // state default
    node_142_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__gate__number::Node node_143 = {
    xod__core__gate__number::State(), // state default
    node_143_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_144 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_144_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__gate__number::Node node_145 = {
    xod__core__gate__number::State(), // state default
    node_145_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__gate__number::Node node_146 = {
    xod__core__gate__number::State(), // state default
    node_146_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_147 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_147_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_148 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_148_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__gate__number::Node node_149 = {
    xod__core__gate__number::State(), // state default
    node_149_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__gate__number::Node node_150 = {
    xod__core__gate__number::State(), // state default
    node_150_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_151 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_151_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_152 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_152_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__gate__number::Node node_153 = {
    xod__core__gate__number::State(), // state default
    node_153_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__gate__number::Node node_154 = {
    xod__core__gate__number::State(), // state default
    node_154_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_155 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_155_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_156 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_156_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__gate__number::Node node_157 = {
    xod__core__gate__number::State(), // state default
    node_157_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__gate__number::Node node_158 = {
    xod__core__gate__number::State(), // state default
    node_158_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_159 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_159_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_160 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_160_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__gate__number::Node node_161 = {
    xod__core__gate__number::State(), // state default
    node_161_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_162 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_162_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__gate__number::Node node_163 = {
    xod__core__gate__number::State(), // state default
    node_163_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_164 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_164_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__gate__number::Node node_165 = {
    xod__core__gate__number::State(), // state default
    node_165_output_OUT, // output OUT default
    true, // OUT dirty
    true // node itself dirty
};
xod__core__cast_to_number__boolean::Node node_166 = {
    xod__core__cast_to_number__boolean::State(), // state default
    node_166_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__any::Node node_167 = {
    xod__core__any::State(), // state default
    node_167_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
nkrkv__af_motor__dc_motors::Node node_168 = {
    nkrkv__af_motor__dc_motors::State(), // state default
    true // node itself dirty
};
nkrkv__af_motor__dc_motors::Node node_169 = {
    nkrkv__af_motor__dc_motors::State(), // state default
    true // node itself dirty
};
nkrkv__af_motor__dc_motors::Node node_170 = {
    nkrkv__af_motor__dc_motors::State(), // state default
    true // node itself dirty
};
xod__core__cube::Node node_171 = {
    xod__core__cube::State(), // state default
    node_171_output_OUT, // output OUT default
    true // node itself dirty
};
nkrkv__af_motor__dc_motors::Node node_172 = {
    nkrkv__af_motor__dc_motors::State(), // state default
    true // node itself dirty
};
xod__core__cube::Node node_173 = {
    xod__core__cube::State(), // state default
    node_173_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__cube::Node node_174 = {
    xod__core__cube::State(), // state default
    node_174_output_OUT, // output OUT default
    true // node itself dirty
};
nkrkv__af_motor__dc_motors::Node node_175 = {
    nkrkv__af_motor__dc_motors::State(), // state default
    true // node itself dirty
};
xod__core__cube::Node node_176 = {
    xod__core__cube::State(), // state default
    node_176_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__cube::Node node_177 = {
    xod__core__cube::State(), // state default
    node_177_output_OUT, // output OUT default
    true // node itself dirty
};
nkrkv__af_motor__dc_motors::Node node_178 = {
    nkrkv__af_motor__dc_motors::State(), // state default
    true // node itself dirty
};
xod__core__cube::Node node_179 = {
    xod__core__cube::State(), // state default
    node_179_output_OUT, // output OUT default
    true // node itself dirty
};
nkrkv__af_motor__dc_motors::Node node_180 = {
    nkrkv__af_motor__dc_motors::State(), // state default
    true // node itself dirty
};
xod__core__cube::Node node_181 = {
    xod__core__cube::State(), // state default
    node_181_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__cube::Node node_182 = {
    xod__core__cube::State(), // state default
    node_182_output_OUT, // output OUT default
    true // node itself dirty
};
nkrkv__af_motor__dc_motors::Node node_183 = {
    nkrkv__af_motor__dc_motors::State(), // state default
    true // node itself dirty
};
nkrkv__af_motor__dc_motors::Node node_184 = {
    nkrkv__af_motor__dc_motors::State(), // state default
    true // node itself dirty
};
nkrkv__af_motor__dc_motors::Node node_185 = {
    nkrkv__af_motor__dc_motors::State(), // state default
    true // node itself dirty
};
xod__core__cube::Node node_186 = {
    xod__core__cube::State(), // state default
    node_186_output_OUT, // output OUT default
    true // node itself dirty
};
nkrkv__af_motor__dc_motors::Node node_187 = {
    nkrkv__af_motor__dc_motors::State(), // state default
    true // node itself dirty
};
xod__core__cube::Node node_188 = {
    xod__core__cube::State(), // state default
    node_188_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__cube::Node node_189 = {
    xod__core__cube::State(), // state default
    node_189_output_OUT, // output OUT default
    true // node itself dirty
};
nkrkv__af_motor__dc_motors::Node node_190 = {
    nkrkv__af_motor__dc_motors::State(), // state default
    true // node itself dirty
};
xod__core__cube::Node node_191 = {
    xod__core__cube::State(), // state default
    node_191_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__cube::Node node_192 = {
    xod__core__cube::State(), // state default
    node_192_output_OUT, // output OUT default
    true // node itself dirty
};
nkrkv__af_motor__dc_motors::Node node_193 = {
    nkrkv__af_motor__dc_motors::State(), // state default
    true // node itself dirty
};
xod__core__cube::Node node_194 = {
    xod__core__cube::State(), // state default
    node_194_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__cube::Node node_195 = {
    xod__core__cube::State(), // state default
    node_195_output_OUT, // output OUT default
    true // node itself dirty
};
nkrkv__af_motor__dc_motors::Node node_196 = {
    nkrkv__af_motor__dc_motors::State(), // state default
    true // node itself dirty
};
xod__core__cube::Node node_197 = {
    xod__core__cube::State(), // state default
    node_197_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__cube::Node node_198 = {
    xod__core__cube::State(), // state default
    node_198_output_OUT, // output OUT default
    true // node itself dirty
};
nkrkv__af_motor__dc_motors::Node node_199 = {
    nkrkv__af_motor__dc_motors::State(), // state default
    true // node itself dirty
};
xod__core__cube::Node node_200 = {
    xod__core__cube::State(), // state default
    node_200_output_OUT, // output OUT default
    true // node itself dirty
};
nkrkv__af_motor__dc_motors::Node node_201 = {
    nkrkv__af_motor__dc_motors::State(), // state default
    true // node itself dirty
};
xod__core__cube::Node node_202 = {
    xod__core__cube::State(), // state default
    node_202_output_OUT, // output OUT default
    true // node itself dirty
};
nkrkv__af_motor__dc_motors::Node node_203 = {
    nkrkv__af_motor__dc_motors::State(), // state default
    true // node itself dirty
};
xod__core__cube::Node node_204 = {
    xod__core__cube::State(), // state default
    node_204_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__clock::Node node_205 = {
    xod__core__clock::State(), // state default
    0, // timeoutAt
    node_205_output_TICK, // output TICK default
    false, // TICK dirty
    true // node itself dirty
};
xod__core__pwm_output::Node node_206 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_207 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_208 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_209 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_210 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_211 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_212 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_213 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_214 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_215 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_216 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_217 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_218 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_219 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_220 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_221 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_222 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_223 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_224 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
xod__core__pwm_output::Node node_225 = {
    xod__core__pwm_output::State(), // state default
    true // node itself dirty
};
gweimer__hc_sr04_ultrasonic__ultrasonic_range::Node node_226 = {
    gweimer__hc_sr04_ultrasonic__ultrasonic_range::State(), // state default
    node_226_output_Dm, // output Dm default
    true, // Dm dirty
    true // node itself dirty
};
xod__core__less::Node node_227 = {
    xod__core__less::State(), // state default
    node_227_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__less::Node node_228 = {
    xod__core__less::State(), // state default
    node_228_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__less::Node node_229 = {
    xod__core__less::State(), // state default
    node_229_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__cast_to_pulse__boolean::Node node_230 = {
    xod__core__cast_to_pulse__boolean::State(), // state default
    node_230_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__cast_to_pulse__boolean::Node node_231 = {
    xod__core__cast_to_pulse__boolean::State(), // state default
    node_231_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__cast_to_pulse__boolean::Node node_232 = {
    xod__core__cast_to_pulse__boolean::State(), // state default
    node_232_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_233 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_233_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_234 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_234_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_235 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_235_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_236 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_236_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_237 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_237_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_238 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_238_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_239 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_239_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_240 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_240_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_241 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_241_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_242 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_242_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_243 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_243_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_244 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_244_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_245 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_245_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_246 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_246_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_247 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_247_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_248 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_248_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_249 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_249_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_250 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_250_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_251 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_251_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_252 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_252_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_253 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_253_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_254 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_254_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_255 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_255_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_256 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_256_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_257 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_257_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_258 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_258_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_259 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_259_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_260 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_260_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};

void runTransaction() {
    g_transactionTime = millis();

    XOD_TRACE_F("Transaction started, t=");
    XOD_TRACE_LN(g_transactionTime);

    // Check for timeouts
    detail::checkTriggerTimeout(&node_105);
    detail::checkTriggerTimeout(&node_106);
    detail::checkTriggerTimeout(&node_107);
    detail::checkTriggerTimeout(&node_108);
    detail::checkTriggerTimeout(&node_109);
    detail::checkTriggerTimeout(&node_110);
    detail::checkTriggerTimeout(&node_111);
    detail::checkTriggerTimeout(&node_112);
    detail::checkTriggerTimeout(&node_113);
    detail::checkTriggerTimeout(&node_114);
    detail::checkTriggerTimeout(&node_115);
    detail::checkTriggerTimeout(&node_116);
    detail::checkTriggerTimeout(&node_117);
    detail::checkTriggerTimeout(&node_118);
    detail::checkTriggerTimeout(&node_119);
    detail::checkTriggerTimeout(&node_120);
    detail::checkTriggerTimeout(&node_121);
    detail::checkTriggerTimeout(&node_205);
    detail::checkTriggerTimeout(&node_233);
    detail::checkTriggerTimeout(&node_234);
    detail::checkTriggerTimeout(&node_235);
    detail::checkTriggerTimeout(&node_236);
    detail::checkTriggerTimeout(&node_237);
    detail::checkTriggerTimeout(&node_238);
    detail::checkTriggerTimeout(&node_239);
    detail::checkTriggerTimeout(&node_240);
    detail::checkTriggerTimeout(&node_241);
    detail::checkTriggerTimeout(&node_242);
    detail::checkTriggerTimeout(&node_243);
    detail::checkTriggerTimeout(&node_244);
    detail::checkTriggerTimeout(&node_245);
    detail::checkTriggerTimeout(&node_246);
    detail::checkTriggerTimeout(&node_247);
    detail::checkTriggerTimeout(&node_248);
    detail::checkTriggerTimeout(&node_249);
    detail::checkTriggerTimeout(&node_250);
    detail::checkTriggerTimeout(&node_251);
    detail::checkTriggerTimeout(&node_252);
    detail::checkTriggerTimeout(&node_253);
    detail::checkTriggerTimeout(&node_254);
    detail::checkTriggerTimeout(&node_255);
    detail::checkTriggerTimeout(&node_256);
    detail::checkTriggerTimeout(&node_257);
    detail::checkTriggerTimeout(&node_258);
    detail::checkTriggerTimeout(&node_259);
    detail::checkTriggerTimeout(&node_260);

    // defer-* nodes are always at the very bottom of the graph, so no one will
    // recieve values emitted by them. We must evaluate them before everybody
    // else to give them a chance to emit values.
    //
    // If trigerred, keep only output dirty, not the node itself, so it will
    // evaluate on the regular pass only if it pushed a new value again.
    {
        if (node_233.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(233);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_233;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_114.isNodeDirty |= node_233.isOutputDirty_OUT;

            node_233.isNodeDirty = false;
            detail::clearTimeout(&node_233);
        }
    }
    {
        if (node_234.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(234);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_234;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_109.isNodeDirty |= node_234.isOutputDirty_OUT;

            node_234.isNodeDirty = false;
            detail::clearTimeout(&node_234);
        }
    }
    {
        if (node_235.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(235);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_235;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_108.isNodeDirty |= node_235.isOutputDirty_OUT;

            node_235.isNodeDirty = false;
            detail::clearTimeout(&node_235);
        }
    }
    {
        if (node_236.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(236);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_236;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_107.isNodeDirty |= node_236.isOutputDirty_OUT;

            node_236.isNodeDirty = false;
            detail::clearTimeout(&node_236);
        }
    }
    {
        if (node_237.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(237);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_237;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_110.isNodeDirty |= node_237.isOutputDirty_OUT;

            node_237.isNodeDirty = false;
            detail::clearTimeout(&node_237);
        }
    }
    {
        if (node_238.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(238);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_238;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_115.isNodeDirty |= node_238.isOutputDirty_OUT;

            node_238.isNodeDirty = false;
            detail::clearTimeout(&node_238);
        }
    }
    {
        if (node_239.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(239);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_239;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_117.isNodeDirty |= node_239.isOutputDirty_OUT;

            node_239.isNodeDirty = false;
            detail::clearTimeout(&node_239);
        }
    }
    {
        if (node_240.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(240);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_240;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty

            node_240.isNodeDirty = false;
            detail::clearTimeout(&node_240);
        }
    }
    {
        if (node_241.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(241);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_241;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_111.isNodeDirty |= node_241.isOutputDirty_OUT;

            node_241.isNodeDirty = false;
            detail::clearTimeout(&node_241);
        }
    }
    {
        if (node_242.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(242);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_242;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_234.isNodeDirty |= node_242.isOutputDirty_OUT;

            node_242.isNodeDirty = false;
            detail::clearTimeout(&node_242);
        }
    }
    {
        if (node_243.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(243);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_243;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_106.isNodeDirty |= node_243.isOutputDirty_OUT;

            node_243.isNodeDirty = false;
            detail::clearTimeout(&node_243);
        }
    }
    {
        if (node_244.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(244);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_244;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_105.isNodeDirty |= node_244.isOutputDirty_OUT;

            node_244.isNodeDirty = false;
            detail::clearTimeout(&node_244);
        }
    }
    {
        if (node_245.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(245);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_245;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_112.isNodeDirty |= node_245.isOutputDirty_OUT;

            node_245.isNodeDirty = false;
            detail::clearTimeout(&node_245);
        }
    }
    {
        if (node_246.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(246);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_246;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_238.isNodeDirty |= node_246.isOutputDirty_OUT;

            node_246.isNodeDirty = false;
            detail::clearTimeout(&node_246);
        }
    }
    {
        if (node_247.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(247);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_247;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_113.isNodeDirty |= node_247.isOutputDirty_OUT;

            node_247.isNodeDirty = false;
            detail::clearTimeout(&node_247);
        }
    }
    {
        if (node_248.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(248);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_248;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_235.isNodeDirty |= node_248.isOutputDirty_OUT;

            node_248.isNodeDirty = false;
            detail::clearTimeout(&node_248);
        }
    }
    {
        if (node_249.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(249);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_249;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty

            node_249.isNodeDirty = false;
            detail::clearTimeout(&node_249);
        }
    }
    {
        if (node_250.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(250);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_250;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_236.isNodeDirty |= node_250.isOutputDirty_OUT;

            node_250.isNodeDirty = false;
            detail::clearTimeout(&node_250);
        }
    }
    {
        if (node_251.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(251);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_251;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_239.isNodeDirty |= node_251.isOutputDirty_OUT;

            node_251.isNodeDirty = false;
            detail::clearTimeout(&node_251);
        }
    }
    {
        if (node_252.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(252);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_252;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_233.isNodeDirty |= node_252.isOutputDirty_OUT;

            node_252.isNodeDirty = false;
            detail::clearTimeout(&node_252);
        }
    }
    {
        if (node_253.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(253);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_253;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_118.isNodeDirty |= node_253.isOutputDirty_OUT;

            node_253.isNodeDirty = false;
            detail::clearTimeout(&node_253);
        }
    }
    {
        if (node_254.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(254);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_254;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_116.isNodeDirty |= node_254.isOutputDirty_OUT;

            node_254.isNodeDirty = false;
            detail::clearTimeout(&node_254);
        }
    }
    {
        if (node_255.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(255);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_255;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_104.isNodeDirty |= node_255.isOutputDirty_OUT;

            node_255.isNodeDirty = false;
            detail::clearTimeout(&node_255);
        }
    }
    {
        if (node_256.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(256);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_256;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_122.isNodeDirty |= node_256.isOutputDirty_OUT;

            node_256.isNodeDirty = false;
            detail::clearTimeout(&node_256);
        }
    }
    {
        if (node_257.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(257);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_257;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_167.isNodeDirty |= node_257.isOutputDirty_OUT;

            node_257.isNodeDirty = false;
            detail::clearTimeout(&node_257);
        }
    }
    {
        if (node_258.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(258);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_258;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_119.isNodeDirty |= node_258.isOutputDirty_OUT;

            node_258.isNodeDirty = false;
            detail::clearTimeout(&node_258);
        }
    }
    {
        if (node_259.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(259);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_259;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_120.isNodeDirty |= node_259.isOutputDirty_OUT;

            node_259.isNodeDirty = false;
            detail::clearTimeout(&node_259);
        }
    }
    {
        if (node_260.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(260);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_260;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_121.isNodeDirty |= node_260.isOutputDirty_OUT;

            node_260.isNodeDirty = false;
            detail::clearTimeout(&node_260);
        }
    }

    // Evaluate all dirty nodes
    { // xod__core__boot #0
        if (node_0.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(0);

            xod__core__boot::ContextObject ctxObj;
            ctxObj._node = &node_0;

            // copy data from upstream nodes into context

            xod__core__boot::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_104.isNodeDirty |= node_0.isOutputDirty_BOOT;
        }
    }
    { // xod__core__any #1
        if (node_1.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(1);

            xod__core__any::ContextObject ctxObj;
            ctxObj._node = &node_1;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = false;
            ctxObj._isInputDirty_IN2 = false;

            xod__core__any::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__any #104
        if (node_104.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(104);

            xod__core__any::ContextObject ctxObj;
            ctxObj._node = &node_104;

            // copy data from upstream nodes into context
            ctxObj._input_IN1 = node_255.output_OUT;
            ctxObj._input_IN2 = node_0.output_BOOT;

            ctxObj._isInputDirty_IN1 = node_255.isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = node_0.isOutputDirty_BOOT;

            xod__core__any::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_122.isNodeDirty |= node_104.isOutputDirty_OUT;
        }
    }
    { // xod__core__delay #105
        if (node_105.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(105);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_105;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_2_output_VAL;
            ctxObj._input_SET = node_244.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_244.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_241.isNodeDirty |= node_105.isOutputDirty_DONE;
            node_123.isNodeDirty |= node_105.isOutputDirty_ACT;
        }
    }
    { // xod__core__delay #106
        if (node_106.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(106);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_106;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_6_output_VAL;
            ctxObj._input_SET = node_243.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_243.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_242.isNodeDirty |= node_106.isOutputDirty_DONE;
            node_124.isNodeDirty |= node_106.isOutputDirty_ACT;
        }
    }
    { // xod__core__delay #107
        if (node_107.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(107);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_107;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_13_output_VAL;
            ctxObj._input_SET = node_236.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_236.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_243.isNodeDirty |= node_107.isOutputDirty_DONE;
            node_125.isNodeDirty |= node_107.isOutputDirty_ACT;
            node_126.isNodeDirty |= node_107.isOutputDirty_ACT;
        }
    }
    { // xod__core__delay #108
        if (node_108.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(108);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_108;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_17_output_VAL;
            ctxObj._input_SET = node_235.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_235.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_244.isNodeDirty |= node_108.isOutputDirty_DONE;
            node_127.isNodeDirty |= node_108.isOutputDirty_ACT;
            node_128.isNodeDirty |= node_108.isOutputDirty_ACT;
            node_129.isNodeDirty |= node_108.isOutputDirty_ACT;
            node_130.isNodeDirty |= node_108.isOutputDirty_ACT;
        }
    }
    { // xod__core__delay #109
        if (node_109.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(109);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_109;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_23_output_VAL;
            ctxObj._input_SET = node_234.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_234.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_245.isNodeDirty |= node_109.isOutputDirty_DONE;
            node_131.isNodeDirty |= node_109.isOutputDirty_ACT;
            node_132.isNodeDirty |= node_109.isOutputDirty_ACT;
            node_133.isNodeDirty |= node_109.isOutputDirty_ACT;
            node_134.isNodeDirty |= node_109.isOutputDirty_ACT;
        }
    }
    { // xod__core__delay #110
        if (node_110.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(110);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_110;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_29_output_VAL;
            ctxObj._input_SET = node_237.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_237.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_246.isNodeDirty |= node_110.isOutputDirty_DONE;
            node_135.isNodeDirty |= node_110.isOutputDirty_ACT;
            node_136.isNodeDirty |= node_110.isOutputDirty_ACT;
        }
    }
    { // xod__core__delay #111
        if (node_111.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(111);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_111;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_35_output_VAL;
            ctxObj._input_SET = node_241.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_241.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_247.isNodeDirty |= node_111.isOutputDirty_DONE;
            node_137.isNodeDirty |= node_111.isOutputDirty_ACT;
            node_138.isNodeDirty |= node_111.isOutputDirty_ACT;
            node_139.isNodeDirty |= node_111.isOutputDirty_ACT;
            node_140.isNodeDirty |= node_111.isOutputDirty_ACT;
        }
    }
    { // xod__core__delay #112
        if (node_112.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(112);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_112;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_41_output_VAL;
            ctxObj._input_SET = node_245.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_245.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_248.isNodeDirty |= node_112.isOutputDirty_DONE;
            node_141.isNodeDirty |= node_112.isOutputDirty_ACT;
        }
    }
    { // xod__core__delay #113
        if (node_113.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(113);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_113;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_45_output_VAL;
            ctxObj._input_SET = node_247.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_247.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_249.isNodeDirty |= node_113.isOutputDirty_DONE;
            node_142.isNodeDirty |= node_113.isOutputDirty_ACT;
        }
    }
    { // xod__core__delay #114
        if (node_114.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(114);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_114;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_49_output_VAL;
            ctxObj._input_SET = node_233.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_233.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_250.isNodeDirty |= node_114.isOutputDirty_DONE;
            node_143.isNodeDirty |= node_114.isOutputDirty_ACT;
            node_144.isNodeDirty |= node_114.isOutputDirty_ACT;
        }
    }
    { // xod__core__delay #115
        if (node_115.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(115);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_115;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_55_output_VAL;
            ctxObj._input_SET = node_238.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_238.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_251.isNodeDirty |= node_115.isOutputDirty_DONE;
            node_145.isNodeDirty |= node_115.isOutputDirty_ACT;
            node_146.isNodeDirty |= node_115.isOutputDirty_ACT;
            node_147.isNodeDirty |= node_115.isOutputDirty_ACT;
            node_148.isNodeDirty |= node_115.isOutputDirty_ACT;
        }
    }
    { // xod__core__delay #116
        if (node_116.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(116);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_116;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_63_output_VAL;
            ctxObj._input_SET = node_254.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_254.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_252.isNodeDirty |= node_116.isOutputDirty_DONE;
            node_149.isNodeDirty |= node_116.isOutputDirty_ACT;
            node_150.isNodeDirty |= node_116.isOutputDirty_ACT;
            node_151.isNodeDirty |= node_116.isOutputDirty_ACT;
            node_152.isNodeDirty |= node_116.isOutputDirty_ACT;
        }
    }
    { // xod__core__delay #117
        if (node_117.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(117);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_117;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_69_output_VAL;
            ctxObj._input_SET = node_239.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_239.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_253.isNodeDirty |= node_117.isOutputDirty_DONE;
            node_153.isNodeDirty |= node_117.isOutputDirty_ACT;
            node_154.isNodeDirty |= node_117.isOutputDirty_ACT;
            node_155.isNodeDirty |= node_117.isOutputDirty_ACT;
            node_156.isNodeDirty |= node_117.isOutputDirty_ACT;
        }
    }
    { // xod__core__delay #118
        if (node_118.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(118);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_118;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_75_output_VAL;
            ctxObj._input_SET = node_253.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_253.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_254.isNodeDirty |= node_118.isOutputDirty_DONE;
            node_157.isNodeDirty |= node_118.isOutputDirty_ACT;
            node_158.isNodeDirty |= node_118.isOutputDirty_ACT;
            node_159.isNodeDirty |= node_118.isOutputDirty_ACT;
            node_160.isNodeDirty |= node_118.isOutputDirty_ACT;
        }
    }
    { // xod__core__delay #119
        if (node_119.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(119);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_119;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_92_output_VAL;
            ctxObj._input_SET = node_258.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_258.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_255.isNodeDirty |= node_119.isOutputDirty_DONE;
            node_161.isNodeDirty |= node_119.isOutputDirty_ACT;
            node_162.isNodeDirty |= node_119.isOutputDirty_ACT;
        }
    }
    { // xod__core__delay #120
        if (node_120.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(120);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_120;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_97_output_VAL;
            ctxObj._input_SET = node_259.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_259.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_256.isNodeDirty |= node_120.isOutputDirty_DONE;
            node_163.isNodeDirty |= node_120.isOutputDirty_ACT;
            node_164.isNodeDirty |= node_120.isOutputDirty_ACT;
        }
    }
    { // xod__core__delay #121
        if (node_121.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(121);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_121;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_99_output_VAL;
            ctxObj._input_SET = node_260.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_260.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_257.isNodeDirty |= node_121.isOutputDirty_DONE;
            node_165.isNodeDirty |= node_121.isOutputDirty_ACT;
            node_166.isNodeDirty |= node_121.isOutputDirty_ACT;
        }
    }
    { // xod__core__any #122
        if (node_122.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(122);

            xod__core__any::ContextObject ctxObj;
            ctxObj._node = &node_122;

            // copy data from upstream nodes into context
            ctxObj._input_IN1 = node_256.output_OUT;
            ctxObj._input_IN2 = node_104.output_OUT;

            ctxObj._isInputDirty_IN1 = node_256.isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = node_104.isOutputDirty_OUT;

            xod__core__any::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_167.isNodeDirty |= node_122.isOutputDirty_OUT;
        }
    }
    { // xod__core__gate__number #123
        if (node_123.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(123);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_123;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_5_output_VAL;
            ctxObj._input_EN = node_105.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_168.isNodeDirty |= node_123.isOutputDirty_OUT;
        }
    }
    { // xod__core__gate__number #124
        if (node_124.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(124);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_124;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_9_output_VAL;
            ctxObj._input_EN = node_106.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_169.isNodeDirty |= node_124.isOutputDirty_OUT;
        }
    }
    { // xod__core__gate__number #125
        if (node_125.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(125);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_125;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_12_output_VAL;
            ctxObj._input_EN = node_107.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_170.isNodeDirty |= node_125.isOutputDirty_OUT;
        }
    }
    { // xod__core__cast_to_number__boolean #126
        if (node_126.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(126);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_126;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_107.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_171.isNodeDirty = true;
        }
    }
    { // xod__core__gate__number #127
        if (node_127.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(127);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_127;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_15_output_VAL;
            ctxObj._input_EN = node_108.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_172.isNodeDirty |= node_127.isOutputDirty_OUT;
        }
    }
    { // xod__core__gate__number #128
        if (node_128.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(128);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_128;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_16_output_VAL;
            ctxObj._input_EN = node_108.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_172.isNodeDirty |= node_128.isOutputDirty_OUT;
        }
    }
    { // xod__core__cast_to_number__boolean #129
        if (node_129.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(129);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_129;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_108.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_173.isNodeDirty = true;
        }
    }
    { // xod__core__cast_to_number__boolean #130
        if (node_130.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(130);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_130;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_108.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_174.isNodeDirty = true;
        }
    }
    { // xod__core__gate__number #131
        if (node_131.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(131);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_131;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_22_output_VAL;
            ctxObj._input_EN = node_109.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_175.isNodeDirty |= node_131.isOutputDirty_OUT;
        }
    }
    { // xod__core__gate__number #132
        if (node_132.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(132);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_132;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_26_output_VAL;
            ctxObj._input_EN = node_109.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_175.isNodeDirty |= node_132.isOutputDirty_OUT;
        }
    }
    { // xod__core__cast_to_number__boolean #133
        if (node_133.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(133);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_133;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_109.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_176.isNodeDirty = true;
        }
    }
    { // xod__core__cast_to_number__boolean #134
        if (node_134.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(134);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_134;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_109.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_177.isNodeDirty = true;
        }
    }
    { // xod__core__gate__number #135
        if (node_135.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(135);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_135;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_30_output_VAL;
            ctxObj._input_EN = node_110.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_178.isNodeDirty |= node_135.isOutputDirty_OUT;
        }
    }
    { // xod__core__cast_to_number__boolean #136
        if (node_136.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(136);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_136;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_110.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_179.isNodeDirty = true;
        }
    }
    { // xod__core__gate__number #137
        if (node_137.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(137);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_137;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_34_output_VAL;
            ctxObj._input_EN = node_111.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_180.isNodeDirty |= node_137.isOutputDirty_OUT;
        }
    }
    { // xod__core__gate__number #138
        if (node_138.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(138);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_138;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_36_output_VAL;
            ctxObj._input_EN = node_111.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_180.isNodeDirty |= node_138.isOutputDirty_OUT;
        }
    }
    { // xod__core__cast_to_number__boolean #139
        if (node_139.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(139);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_139;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_111.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_181.isNodeDirty = true;
        }
    }
    { // xod__core__cast_to_number__boolean #140
        if (node_140.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(140);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_140;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_111.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_182.isNodeDirty = true;
        }
    }
    { // xod__core__gate__number #141
        if (node_141.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(141);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_141;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_44_output_VAL;
            ctxObj._input_EN = node_112.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_183.isNodeDirty |= node_141.isOutputDirty_OUT;
        }
    }
    { // xod__core__gate__number #142
        if (node_142.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(142);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_142;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_48_output_VAL;
            ctxObj._input_EN = node_113.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_184.isNodeDirty |= node_142.isOutputDirty_OUT;
        }
    }
    { // xod__core__gate__number #143
        if (node_143.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(143);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_143;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_50_output_VAL;
            ctxObj._input_EN = node_114.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_185.isNodeDirty |= node_143.isOutputDirty_OUT;
        }
    }
    { // xod__core__cast_to_number__boolean #144
        if (node_144.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(144);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_144;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_114.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_186.isNodeDirty = true;
        }
    }
    { // xod__core__gate__number #145
        if (node_145.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(145);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_145;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_54_output_VAL;
            ctxObj._input_EN = node_115.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_187.isNodeDirty |= node_145.isOutputDirty_OUT;
        }
    }
    { // xod__core__gate__number #146
        if (node_146.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(146);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_146;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_58_output_VAL;
            ctxObj._input_EN = node_115.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_187.isNodeDirty |= node_146.isOutputDirty_OUT;
        }
    }
    { // xod__core__cast_to_number__boolean #147
        if (node_147.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(147);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_147;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_115.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_188.isNodeDirty = true;
        }
    }
    { // xod__core__cast_to_number__boolean #148
        if (node_148.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(148);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_148;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_115.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_189.isNodeDirty = true;
        }
    }
    { // xod__core__gate__number #149
        if (node_149.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(149);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_149;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_61_output_VAL;
            ctxObj._input_EN = node_116.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_190.isNodeDirty |= node_149.isOutputDirty_OUT;
        }
    }
    { // xod__core__gate__number #150
        if (node_150.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(150);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_150;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_62_output_VAL;
            ctxObj._input_EN = node_116.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_190.isNodeDirty |= node_150.isOutputDirty_OUT;
        }
    }
    { // xod__core__cast_to_number__boolean #151
        if (node_151.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(151);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_151;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_116.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_191.isNodeDirty = true;
        }
    }
    { // xod__core__cast_to_number__boolean #152
        if (node_152.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(152);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_152;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_116.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_192.isNodeDirty = true;
        }
    }
    { // xod__core__gate__number #153
        if (node_153.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(153);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_153;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_68_output_VAL;
            ctxObj._input_EN = node_117.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_193.isNodeDirty |= node_153.isOutputDirty_OUT;
        }
    }
    { // xod__core__gate__number #154
        if (node_154.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(154);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_154;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_70_output_VAL;
            ctxObj._input_EN = node_117.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_193.isNodeDirty |= node_154.isOutputDirty_OUT;
        }
    }
    { // xod__core__cast_to_number__boolean #155
        if (node_155.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(155);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_155;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_117.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_194.isNodeDirty = true;
        }
    }
    { // xod__core__cast_to_number__boolean #156
        if (node_156.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(156);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_156;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_117.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_195.isNodeDirty = true;
        }
    }
    { // xod__core__gate__number #157
        if (node_157.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(157);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_157;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_76_output_VAL;
            ctxObj._input_EN = node_118.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_196.isNodeDirty |= node_157.isOutputDirty_OUT;
        }
    }
    { // xod__core__gate__number #158
        if (node_158.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(158);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_158;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_79_output_VAL;
            ctxObj._input_EN = node_118.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_196.isNodeDirty |= node_158.isOutputDirty_OUT;
        }
    }
    { // xod__core__cast_to_number__boolean #159
        if (node_159.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(159);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_159;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_118.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_197.isNodeDirty = true;
        }
    }
    { // xod__core__cast_to_number__boolean #160
        if (node_160.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(160);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_160;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_118.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_198.isNodeDirty = true;
        }
    }
    { // xod__core__gate__number #161
        if (node_161.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(161);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_161;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_91_output_VAL;
            ctxObj._input_EN = node_119.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_199.isNodeDirty |= node_161.isOutputDirty_OUT;
        }
    }
    { // xod__core__cast_to_number__boolean #162
        if (node_162.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(162);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_162;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_119.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_200.isNodeDirty = true;
        }
    }
    { // xod__core__gate__number #163
        if (node_163.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(163);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_163;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_96_output_VAL;
            ctxObj._input_EN = node_120.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_201.isNodeDirty |= node_163.isOutputDirty_OUT;
        }
    }
    { // xod__core__cast_to_number__boolean #164
        if (node_164.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(164);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_164;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_120.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_202.isNodeDirty = true;
        }
    }
    { // xod__core__gate__number #165
        if (node_165.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(165);

            xod__core__gate__number::ContextObject ctxObj;
            ctxObj._node = &node_165;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_100_output_VAL;
            ctxObj._input_EN = node_121.output_ACT;

            xod__core__gate__number::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_203.isNodeDirty |= node_165.isOutputDirty_OUT;
        }
    }
    { // xod__core__cast_to_number__boolean #166
        if (node_166.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(166);

            xod__core__cast_to_number__boolean::ContextObject ctxObj;
            ctxObj._node = &node_166;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_121.output_ACT;

            xod__core__cast_to_number__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_204.isNodeDirty = true;
        }
    }
    { // xod__core__any #167
        if (node_167.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(167);

            xod__core__any::ContextObject ctxObj;
            ctxObj._node = &node_167;

            // copy data from upstream nodes into context
            ctxObj._input_IN1 = node_257.output_OUT;
            ctxObj._input_IN2 = node_122.output_OUT;

            ctxObj._isInputDirty_IN1 = node_257.isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = node_122.isOutputDirty_OUT;

            xod__core__any::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_205.isNodeDirty |= node_167.isOutputDirty_OUT;
        }
    }
    { // nkrkv__af_motor__dc_motors #168
        if (node_168.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(168);

            nkrkv__af_motor__dc_motors::ContextObject ctxObj;
            ctxObj._node = &node_168;

            // copy data from upstream nodes into context
            ctxObj._input_M1 = node_123.output_OUT;
            ctxObj._input_M2 = node_123.output_OUT;
            ctxObj._input_M3 = node_3_output_VAL;
            ctxObj._input_M4 = node_4_output_VAL;

            nkrkv__af_motor__dc_motors::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // nkrkv__af_motor__dc_motors #169
        if (node_169.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(169);

            nkrkv__af_motor__dc_motors::ContextObject ctxObj;
            ctxObj._node = &node_169;

            // copy data from upstream nodes into context
            ctxObj._input_M1 = node_124.output_OUT;
            ctxObj._input_M2 = node_124.output_OUT;
            ctxObj._input_M3 = node_7_output_VAL;
            ctxObj._input_M4 = node_8_output_VAL;

            nkrkv__af_motor__dc_motors::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // nkrkv__af_motor__dc_motors #170
        if (node_170.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(170);

            nkrkv__af_motor__dc_motors::ContextObject ctxObj;
            ctxObj._node = &node_170;

            // copy data from upstream nodes into context
            ctxObj._input_M1 = node_125.output_OUT;
            ctxObj._input_M2 = node_125.output_OUT;
            ctxObj._input_M3 = node_10_output_VAL;
            ctxObj._input_M4 = node_11_output_VAL;

            nkrkv__af_motor__dc_motors::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__cube #171
        if (node_171.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(171);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_171;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_126.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_206.isNodeDirty = true;
        }
    }
    { // nkrkv__af_motor__dc_motors #172
        if (node_172.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(172);

            nkrkv__af_motor__dc_motors::ContextObject ctxObj;
            ctxObj._node = &node_172;

            // copy data from upstream nodes into context
            ctxObj._input_M1 = node_127.output_OUT;
            ctxObj._input_M2 = node_128.output_OUT;
            ctxObj._input_M3 = node_18_output_VAL;
            ctxObj._input_M4 = node_19_output_VAL;

            nkrkv__af_motor__dc_motors::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__cube #173
        if (node_173.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(173);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_173;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_129.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_207.isNodeDirty = true;
        }
    }
    { // xod__core__cube #174
        if (node_174.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(174);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_174;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_130.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_208.isNodeDirty = true;
        }
    }
    { // nkrkv__af_motor__dc_motors #175
        if (node_175.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(175);

            nkrkv__af_motor__dc_motors::ContextObject ctxObj;
            ctxObj._node = &node_175;

            // copy data from upstream nodes into context
            ctxObj._input_M1 = node_132.output_OUT;
            ctxObj._input_M2 = node_131.output_OUT;
            ctxObj._input_M3 = node_24_output_VAL;
            ctxObj._input_M4 = node_25_output_VAL;

            nkrkv__af_motor__dc_motors::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__cube #176
        if (node_176.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(176);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_176;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_133.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_209.isNodeDirty = true;
        }
    }
    { // xod__core__cube #177
        if (node_177.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(177);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_177;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_134.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_210.isNodeDirty = true;
        }
    }
    { // nkrkv__af_motor__dc_motors #178
        if (node_178.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(178);

            nkrkv__af_motor__dc_motors::ContextObject ctxObj;
            ctxObj._node = &node_178;

            // copy data from upstream nodes into context
            ctxObj._input_M1 = node_135.output_OUT;
            ctxObj._input_M2 = node_135.output_OUT;
            ctxObj._input_M3 = node_31_output_VAL;
            ctxObj._input_M4 = node_32_output_VAL;

            nkrkv__af_motor__dc_motors::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__cube #179
        if (node_179.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(179);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_179;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_136.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_211.isNodeDirty = true;
        }
    }
    { // nkrkv__af_motor__dc_motors #180
        if (node_180.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(180);

            nkrkv__af_motor__dc_motors::ContextObject ctxObj;
            ctxObj._node = &node_180;

            // copy data from upstream nodes into context
            ctxObj._input_M1 = node_137.output_OUT;
            ctxObj._input_M2 = node_138.output_OUT;
            ctxObj._input_M3 = node_37_output_VAL;
            ctxObj._input_M4 = node_38_output_VAL;

            nkrkv__af_motor__dc_motors::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__cube #181
        if (node_181.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(181);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_181;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_139.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_212.isNodeDirty = true;
        }
    }
    { // xod__core__cube #182
        if (node_182.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(182);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_182;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_140.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_213.isNodeDirty = true;
        }
    }
    { // nkrkv__af_motor__dc_motors #183
        if (node_183.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(183);

            nkrkv__af_motor__dc_motors::ContextObject ctxObj;
            ctxObj._node = &node_183;

            // copy data from upstream nodes into context
            ctxObj._input_M1 = node_141.output_OUT;
            ctxObj._input_M2 = node_141.output_OUT;
            ctxObj._input_M3 = node_42_output_VAL;
            ctxObj._input_M4 = node_43_output_VAL;

            nkrkv__af_motor__dc_motors::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // nkrkv__af_motor__dc_motors #184
        if (node_184.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(184);

            nkrkv__af_motor__dc_motors::ContextObject ctxObj;
            ctxObj._node = &node_184;

            // copy data from upstream nodes into context
            ctxObj._input_M1 = node_142.output_OUT;
            ctxObj._input_M2 = node_142.output_OUT;
            ctxObj._input_M3 = node_46_output_VAL;
            ctxObj._input_M4 = node_47_output_VAL;

            nkrkv__af_motor__dc_motors::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // nkrkv__af_motor__dc_motors #185
        if (node_185.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(185);

            nkrkv__af_motor__dc_motors::ContextObject ctxObj;
            ctxObj._node = &node_185;

            // copy data from upstream nodes into context
            ctxObj._input_M1 = node_143.output_OUT;
            ctxObj._input_M2 = node_143.output_OUT;
            ctxObj._input_M3 = node_51_output_VAL;
            ctxObj._input_M4 = node_52_output_VAL;

            nkrkv__af_motor__dc_motors::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__cube #186
        if (node_186.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(186);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_186;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_144.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_214.isNodeDirty = true;
        }
    }
    { // nkrkv__af_motor__dc_motors #187
        if (node_187.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(187);

            nkrkv__af_motor__dc_motors::ContextObject ctxObj;
            ctxObj._node = &node_187;

            // copy data from upstream nodes into context
            ctxObj._input_M1 = node_146.output_OUT;
            ctxObj._input_M2 = node_145.output_OUT;
            ctxObj._input_M3 = node_56_output_VAL;
            ctxObj._input_M4 = node_57_output_VAL;

            nkrkv__af_motor__dc_motors::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__cube #188
        if (node_188.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(188);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_188;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_147.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_215.isNodeDirty = true;
        }
    }
    { // xod__core__cube #189
        if (node_189.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(189);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_189;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_148.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_216.isNodeDirty = true;
        }
    }
    { // nkrkv__af_motor__dc_motors #190
        if (node_190.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(190);

            nkrkv__af_motor__dc_motors::ContextObject ctxObj;
            ctxObj._node = &node_190;

            // copy data from upstream nodes into context
            ctxObj._input_M1 = node_149.output_OUT;
            ctxObj._input_M2 = node_150.output_OUT;
            ctxObj._input_M3 = node_64_output_VAL;
            ctxObj._input_M4 = node_65_output_VAL;

            nkrkv__af_motor__dc_motors::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__cube #191
        if (node_191.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(191);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_191;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_151.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_217.isNodeDirty = true;
        }
    }
    { // xod__core__cube #192
        if (node_192.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(192);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_192;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_152.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_218.isNodeDirty = true;
        }
    }
    { // nkrkv__af_motor__dc_motors #193
        if (node_193.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(193);

            nkrkv__af_motor__dc_motors::ContextObject ctxObj;
            ctxObj._node = &node_193;

            // copy data from upstream nodes into context
            ctxObj._input_M1 = node_153.output_OUT;
            ctxObj._input_M2 = node_154.output_OUT;
            ctxObj._input_M3 = node_71_output_VAL;
            ctxObj._input_M4 = node_72_output_VAL;

            nkrkv__af_motor__dc_motors::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__cube #194
        if (node_194.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(194);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_194;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_155.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_219.isNodeDirty = true;
        }
    }
    { // xod__core__cube #195
        if (node_195.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(195);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_195;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_156.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_220.isNodeDirty = true;
        }
    }
    { // nkrkv__af_motor__dc_motors #196
        if (node_196.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(196);

            nkrkv__af_motor__dc_motors::ContextObject ctxObj;
            ctxObj._node = &node_196;

            // copy data from upstream nodes into context
            ctxObj._input_M1 = node_158.output_OUT;
            ctxObj._input_M2 = node_157.output_OUT;
            ctxObj._input_M3 = node_77_output_VAL;
            ctxObj._input_M4 = node_78_output_VAL;

            nkrkv__af_motor__dc_motors::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__cube #197
        if (node_197.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(197);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_197;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_159.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_221.isNodeDirty = true;
        }
    }
    { // xod__core__cube #198
        if (node_198.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(198);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_198;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_160.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_222.isNodeDirty = true;
        }
    }
    { // nkrkv__af_motor__dc_motors #199
        if (node_199.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(199);

            nkrkv__af_motor__dc_motors::ContextObject ctxObj;
            ctxObj._node = &node_199;

            // copy data from upstream nodes into context
            ctxObj._input_M1 = node_161.output_OUT;
            ctxObj._input_M2 = node_161.output_OUT;
            ctxObj._input_M3 = node_89_output_VAL;
            ctxObj._input_M4 = node_90_output_VAL;

            nkrkv__af_motor__dc_motors::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__cube #200
        if (node_200.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(200);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_200;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_162.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_223.isNodeDirty = true;
        }
    }
    { // nkrkv__af_motor__dc_motors #201
        if (node_201.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(201);

            nkrkv__af_motor__dc_motors::ContextObject ctxObj;
            ctxObj._node = &node_201;

            // copy data from upstream nodes into context
            ctxObj._input_M1 = node_163.output_OUT;
            ctxObj._input_M2 = node_163.output_OUT;
            ctxObj._input_M3 = node_94_output_VAL;
            ctxObj._input_M4 = node_95_output_VAL;

            nkrkv__af_motor__dc_motors::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__cube #202
        if (node_202.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(202);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_202;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_164.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_224.isNodeDirty = true;
        }
    }
    { // nkrkv__af_motor__dc_motors #203
        if (node_203.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(203);

            nkrkv__af_motor__dc_motors::ContextObject ctxObj;
            ctxObj._node = &node_203;

            // copy data from upstream nodes into context
            ctxObj._input_M1 = node_165.output_OUT;
            ctxObj._input_M2 = node_165.output_OUT;
            ctxObj._input_M3 = node_101_output_VAL;
            ctxObj._input_M4 = node_102_output_VAL;

            nkrkv__af_motor__dc_motors::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__cube #204
        if (node_204.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(204);

            xod__core__cube::ContextObject ctxObj;
            ctxObj._node = &node_204;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_166.output_OUT;

            xod__core__cube::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_225.isNodeDirty = true;
        }
    }
    { // xod__core__clock #205
        if (node_205.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(205);

            xod__core__clock::ContextObject ctxObj;
            ctxObj._node = &node_205;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_87_output_VAL;
            ctxObj._input_IVAL = node_88_output_VAL;
            ctxObj._input_RST = node_167.output_OUT;

            ctxObj._isInputDirty_EN = g_isSettingUp;
            ctxObj._isInputDirty_RST = node_167.isOutputDirty_OUT;

            xod__core__clock::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_226.isNodeDirty |= node_205.isOutputDirty_TICK;
        }
    }
    { // xod__core__pwm_output #206
        if (node_206.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(206);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_206;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_14_output_VAL;
            ctxObj._input_DUTY = node_171.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #207
        if (node_207.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(207);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_207;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_20_output_VAL;
            ctxObj._input_DUTY = node_173.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #208
        if (node_208.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(208);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_208;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_21_output_VAL;
            ctxObj._input_DUTY = node_174.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #209
        if (node_209.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(209);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_209;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_27_output_VAL;
            ctxObj._input_DUTY = node_176.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #210
        if (node_210.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(210);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_210;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_28_output_VAL;
            ctxObj._input_DUTY = node_177.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #211
        if (node_211.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(211);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_211;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_33_output_VAL;
            ctxObj._input_DUTY = node_179.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #212
        if (node_212.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(212);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_212;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_39_output_VAL;
            ctxObj._input_DUTY = node_181.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #213
        if (node_213.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(213);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_213;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_40_output_VAL;
            ctxObj._input_DUTY = node_182.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #214
        if (node_214.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(214);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_214;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_53_output_VAL;
            ctxObj._input_DUTY = node_186.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #215
        if (node_215.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(215);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_215;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_59_output_VAL;
            ctxObj._input_DUTY = node_188.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #216
        if (node_216.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(216);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_216;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_60_output_VAL;
            ctxObj._input_DUTY = node_189.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #217
        if (node_217.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(217);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_217;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_66_output_VAL;
            ctxObj._input_DUTY = node_191.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #218
        if (node_218.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(218);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_218;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_67_output_VAL;
            ctxObj._input_DUTY = node_192.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #219
        if (node_219.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(219);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_219;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_73_output_VAL;
            ctxObj._input_DUTY = node_194.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #220
        if (node_220.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(220);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_220;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_74_output_VAL;
            ctxObj._input_DUTY = node_195.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #221
        if (node_221.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(221);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_221;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_80_output_VAL;
            ctxObj._input_DUTY = node_197.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #222
        if (node_222.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(222);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_222;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_81_output_VAL;
            ctxObj._input_DUTY = node_198.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #223
        if (node_223.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(223);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_223;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_93_output_VAL;
            ctxObj._input_DUTY = node_200.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #224
        if (node_224.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(224);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_224;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_98_output_VAL;
            ctxObj._input_DUTY = node_202.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__pwm_output #225
        if (node_225.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(225);

            xod__core__pwm_output::ContextObject ctxObj;
            ctxObj._node = &node_225;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_103_output_VAL;
            ctxObj._input_DUTY = node_204.output_OUT;

            xod__core__pwm_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // gweimer__hc_sr04_ultrasonic__ultrasonic_range #226
        if (node_226.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(226);

            gweimer__hc_sr04_ultrasonic__ultrasonic_range::ContextObject ctxObj;
            ctxObj._node = &node_226;

            // copy data from upstream nodes into context
            ctxObj._input_TRIG = node_84_output_VAL;
            ctxObj._input_ECHO = node_85_output_VAL;
            ctxObj._input_PING = node_205.output_TICK;

            ctxObj._isInputDirty_PING = node_205.isOutputDirty_TICK;

            gweimer__hc_sr04_ultrasonic__ultrasonic_range::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_228.isNodeDirty |= node_226.isOutputDirty_Dm;
            node_227.isNodeDirty |= node_226.isOutputDirty_Dm;
            node_229.isNodeDirty |= node_226.isOutputDirty_Dm;
        }
    }
    { // xod__core__less #227
        if (node_227.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(227);

            xod__core__less::ContextObject ctxObj;
            ctxObj._node = &node_227;

            // copy data from upstream nodes into context
            ctxObj._input_IN1 = node_226.output_Dm;
            ctxObj._input_IN2 = node_82_output_VAL;

            xod__core__less::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_230.isNodeDirty = true;
        }
    }
    { // xod__core__less #228
        if (node_228.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(228);

            xod__core__less::ContextObject ctxObj;
            ctxObj._node = &node_228;

            // copy data from upstream nodes into context
            ctxObj._input_IN1 = node_226.output_Dm;
            ctxObj._input_IN2 = node_83_output_VAL;

            xod__core__less::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_231.isNodeDirty = true;
        }
    }
    { // xod__core__less #229
        if (node_229.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(229);

            xod__core__less::ContextObject ctxObj;
            ctxObj._node = &node_229;

            // copy data from upstream nodes into context
            ctxObj._input_IN1 = node_226.output_Dm;
            ctxObj._input_IN2 = node_86_output_VAL;

            xod__core__less::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_232.isNodeDirty = true;
        }
    }
    { // xod__core__cast_to_pulse__boolean #230
        if (node_230.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(230);

            xod__core__cast_to_pulse__boolean::ContextObject ctxObj;
            ctxObj._node = &node_230;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_227.output_OUT;

            xod__core__cast_to_pulse__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_258.isNodeDirty |= node_230.isOutputDirty_OUT;
        }
    }
    { // xod__core__cast_to_pulse__boolean #231
        if (node_231.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(231);

            xod__core__cast_to_pulse__boolean::ContextObject ctxObj;
            ctxObj._node = &node_231;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_228.output_OUT;

            xod__core__cast_to_pulse__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_259.isNodeDirty |= node_231.isOutputDirty_OUT;
        }
    }
    { // xod__core__cast_to_pulse__boolean #232
        if (node_232.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(232);

            xod__core__cast_to_pulse__boolean::ContextObject ctxObj;
            ctxObj._node = &node_232;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_229.output_OUT;

            xod__core__cast_to_pulse__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_260.isNodeDirty |= node_232.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #233
        if (node_233.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(233);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_233;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_252.output_OUT;

            ctxObj._isInputDirty_IN = node_252.isOutputDirty_OUT;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_114.isNodeDirty |= node_233.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #234
        if (node_234.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(234);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_234;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_242.output_OUT;

            ctxObj._isInputDirty_IN = node_242.isOutputDirty_OUT;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_109.isNodeDirty |= node_234.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #235
        if (node_235.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(235);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_235;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_248.output_OUT;

            ctxObj._isInputDirty_IN = node_248.isOutputDirty_OUT;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_108.isNodeDirty |= node_235.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #236
        if (node_236.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(236);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_236;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_250.output_OUT;

            ctxObj._isInputDirty_IN = node_250.isOutputDirty_OUT;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_107.isNodeDirty |= node_236.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #237
        if (node_237.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(237);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_237;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_110.isNodeDirty |= node_237.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #238
        if (node_238.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(238);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_238;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_246.output_OUT;

            ctxObj._isInputDirty_IN = node_246.isOutputDirty_OUT;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_115.isNodeDirty |= node_238.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #239
        if (node_239.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(239);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_239;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_251.output_OUT;

            ctxObj._isInputDirty_IN = node_251.isOutputDirty_OUT;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_117.isNodeDirty |= node_239.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #240
        if (node_240.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(240);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_240;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__defer__pulse #241
        if (node_241.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(241);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_241;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_105.output_DONE;

            ctxObj._isInputDirty_IN = node_105.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_111.isNodeDirty |= node_241.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #242
        if (node_242.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(242);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_242;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_106.output_DONE;

            ctxObj._isInputDirty_IN = node_106.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_234.isNodeDirty |= node_242.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #243
        if (node_243.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(243);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_243;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_107.output_DONE;

            ctxObj._isInputDirty_IN = node_107.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_106.isNodeDirty |= node_243.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #244
        if (node_244.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(244);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_244;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_108.output_DONE;

            ctxObj._isInputDirty_IN = node_108.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_105.isNodeDirty |= node_244.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #245
        if (node_245.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(245);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_245;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_109.output_DONE;

            ctxObj._isInputDirty_IN = node_109.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_112.isNodeDirty |= node_245.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #246
        if (node_246.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(246);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_246;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_110.output_DONE;

            ctxObj._isInputDirty_IN = node_110.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_238.isNodeDirty |= node_246.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #247
        if (node_247.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(247);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_247;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_111.output_DONE;

            ctxObj._isInputDirty_IN = node_111.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_113.isNodeDirty |= node_247.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #248
        if (node_248.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(248);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_248;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_112.output_DONE;

            ctxObj._isInputDirty_IN = node_112.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_235.isNodeDirty |= node_248.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #249
        if (node_249.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(249);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_249;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_113.output_DONE;

            ctxObj._isInputDirty_IN = node_113.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__defer__pulse #250
        if (node_250.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(250);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_250;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_114.output_DONE;

            ctxObj._isInputDirty_IN = node_114.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_236.isNodeDirty |= node_250.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #251
        if (node_251.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(251);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_251;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_115.output_DONE;

            ctxObj._isInputDirty_IN = node_115.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_239.isNodeDirty |= node_251.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #252
        if (node_252.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(252);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_252;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_116.output_DONE;

            ctxObj._isInputDirty_IN = node_116.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_233.isNodeDirty |= node_252.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #253
        if (node_253.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(253);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_253;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_117.output_DONE;

            ctxObj._isInputDirty_IN = node_117.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_118.isNodeDirty |= node_253.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #254
        if (node_254.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(254);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_254;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_118.output_DONE;

            ctxObj._isInputDirty_IN = node_118.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_116.isNodeDirty |= node_254.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #255
        if (node_255.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(255);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_255;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_119.output_DONE;

            ctxObj._isInputDirty_IN = node_119.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_104.isNodeDirty |= node_255.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #256
        if (node_256.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(256);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_256;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_120.output_DONE;

            ctxObj._isInputDirty_IN = node_120.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_122.isNodeDirty |= node_256.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #257
        if (node_257.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(257);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_257;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_121.output_DONE;

            ctxObj._isInputDirty_IN = node_121.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_167.isNodeDirty |= node_257.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #258
        if (node_258.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(258);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_258;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_230.output_OUT;

            ctxObj._isInputDirty_IN = node_230.isOutputDirty_OUT;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_119.isNodeDirty |= node_258.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #259
        if (node_259.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(259);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_259;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_231.output_OUT;

            ctxObj._isInputDirty_IN = node_231.isOutputDirty_OUT;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_120.isNodeDirty |= node_259.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #260
        if (node_260.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(260);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_260;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_232.output_OUT;

            ctxObj._isInputDirty_IN = node_232.isOutputDirty_OUT;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_121.isNodeDirty |= node_260.isOutputDirty_OUT;
        }
    }

    // Clear dirtieness and timeouts for all nodes and pins
    node_0.dirtyFlags = 0;
    node_1.dirtyFlags = 0;
    node_104.dirtyFlags = 0;
    node_105.dirtyFlags = 0;
    node_106.dirtyFlags = 0;
    node_107.dirtyFlags = 0;
    node_108.dirtyFlags = 0;
    node_109.dirtyFlags = 0;
    node_110.dirtyFlags = 0;
    node_111.dirtyFlags = 0;
    node_112.dirtyFlags = 0;
    node_113.dirtyFlags = 0;
    node_114.dirtyFlags = 0;
    node_115.dirtyFlags = 0;
    node_116.dirtyFlags = 0;
    node_117.dirtyFlags = 0;
    node_118.dirtyFlags = 0;
    node_119.dirtyFlags = 0;
    node_120.dirtyFlags = 0;
    node_121.dirtyFlags = 0;
    node_122.dirtyFlags = 0;
    node_123.dirtyFlags = 0;
    node_124.dirtyFlags = 0;
    node_125.dirtyFlags = 0;
    node_126.dirtyFlags = 0;
    node_127.dirtyFlags = 0;
    node_128.dirtyFlags = 0;
    node_129.dirtyFlags = 0;
    node_130.dirtyFlags = 0;
    node_131.dirtyFlags = 0;
    node_132.dirtyFlags = 0;
    node_133.dirtyFlags = 0;
    node_134.dirtyFlags = 0;
    node_135.dirtyFlags = 0;
    node_136.dirtyFlags = 0;
    node_137.dirtyFlags = 0;
    node_138.dirtyFlags = 0;
    node_139.dirtyFlags = 0;
    node_140.dirtyFlags = 0;
    node_141.dirtyFlags = 0;
    node_142.dirtyFlags = 0;
    node_143.dirtyFlags = 0;
    node_144.dirtyFlags = 0;
    node_145.dirtyFlags = 0;
    node_146.dirtyFlags = 0;
    node_147.dirtyFlags = 0;
    node_148.dirtyFlags = 0;
    node_149.dirtyFlags = 0;
    node_150.dirtyFlags = 0;
    node_151.dirtyFlags = 0;
    node_152.dirtyFlags = 0;
    node_153.dirtyFlags = 0;
    node_154.dirtyFlags = 0;
    node_155.dirtyFlags = 0;
    node_156.dirtyFlags = 0;
    node_157.dirtyFlags = 0;
    node_158.dirtyFlags = 0;
    node_159.dirtyFlags = 0;
    node_160.dirtyFlags = 0;
    node_161.dirtyFlags = 0;
    node_162.dirtyFlags = 0;
    node_163.dirtyFlags = 0;
    node_164.dirtyFlags = 0;
    node_165.dirtyFlags = 0;
    node_166.dirtyFlags = 0;
    node_167.dirtyFlags = 0;
    node_168.dirtyFlags = 0;
    node_169.dirtyFlags = 0;
    node_170.dirtyFlags = 0;
    node_171.dirtyFlags = 0;
    node_172.dirtyFlags = 0;
    node_173.dirtyFlags = 0;
    node_174.dirtyFlags = 0;
    node_175.dirtyFlags = 0;
    node_176.dirtyFlags = 0;
    node_177.dirtyFlags = 0;
    node_178.dirtyFlags = 0;
    node_179.dirtyFlags = 0;
    node_180.dirtyFlags = 0;
    node_181.dirtyFlags = 0;
    node_182.dirtyFlags = 0;
    node_183.dirtyFlags = 0;
    node_184.dirtyFlags = 0;
    node_185.dirtyFlags = 0;
    node_186.dirtyFlags = 0;
    node_187.dirtyFlags = 0;
    node_188.dirtyFlags = 0;
    node_189.dirtyFlags = 0;
    node_190.dirtyFlags = 0;
    node_191.dirtyFlags = 0;
    node_192.dirtyFlags = 0;
    node_193.dirtyFlags = 0;
    node_194.dirtyFlags = 0;
    node_195.dirtyFlags = 0;
    node_196.dirtyFlags = 0;
    node_197.dirtyFlags = 0;
    node_198.dirtyFlags = 0;
    node_199.dirtyFlags = 0;
    node_200.dirtyFlags = 0;
    node_201.dirtyFlags = 0;
    node_202.dirtyFlags = 0;
    node_203.dirtyFlags = 0;
    node_204.dirtyFlags = 0;
    node_205.dirtyFlags = 0;
    node_206.dirtyFlags = 0;
    node_207.dirtyFlags = 0;
    node_208.dirtyFlags = 0;
    node_209.dirtyFlags = 0;
    node_210.dirtyFlags = 0;
    node_211.dirtyFlags = 0;
    node_212.dirtyFlags = 0;
    node_213.dirtyFlags = 0;
    node_214.dirtyFlags = 0;
    node_215.dirtyFlags = 0;
    node_216.dirtyFlags = 0;
    node_217.dirtyFlags = 0;
    node_218.dirtyFlags = 0;
    node_219.dirtyFlags = 0;
    node_220.dirtyFlags = 0;
    node_221.dirtyFlags = 0;
    node_222.dirtyFlags = 0;
    node_223.dirtyFlags = 0;
    node_224.dirtyFlags = 0;
    node_225.dirtyFlags = 0;
    node_226.dirtyFlags = 0;
    node_227.dirtyFlags = 0;
    node_228.dirtyFlags = 0;
    node_229.dirtyFlags = 0;
    node_230.dirtyFlags = 0;
    node_231.dirtyFlags = 0;
    node_232.dirtyFlags = 0;
    node_233.dirtyFlags = 0;
    node_234.dirtyFlags = 0;
    node_235.dirtyFlags = 0;
    node_236.dirtyFlags = 0;
    node_237.dirtyFlags = 0;
    node_238.dirtyFlags = 0;
    node_239.dirtyFlags = 0;
    node_240.dirtyFlags = 0;
    node_241.dirtyFlags = 0;
    node_242.dirtyFlags = 0;
    node_243.dirtyFlags = 0;
    node_244.dirtyFlags = 0;
    node_245.dirtyFlags = 0;
    node_246.dirtyFlags = 0;
    node_247.dirtyFlags = 0;
    node_248.dirtyFlags = 0;
    node_249.dirtyFlags = 0;
    node_250.dirtyFlags = 0;
    node_251.dirtyFlags = 0;
    node_252.dirtyFlags = 0;
    node_253.dirtyFlags = 0;
    node_254.dirtyFlags = 0;
    node_255.dirtyFlags = 0;
    node_256.dirtyFlags = 0;
    node_257.dirtyFlags = 0;
    node_258.dirtyFlags = 0;
    node_259.dirtyFlags = 0;
    node_260.dirtyFlags = 0;
    detail::clearStaleTimeout(&node_105);
    detail::clearStaleTimeout(&node_106);
    detail::clearStaleTimeout(&node_107);
    detail::clearStaleTimeout(&node_108);
    detail::clearStaleTimeout(&node_109);
    detail::clearStaleTimeout(&node_110);
    detail::clearStaleTimeout(&node_111);
    detail::clearStaleTimeout(&node_112);
    detail::clearStaleTimeout(&node_113);
    detail::clearStaleTimeout(&node_114);
    detail::clearStaleTimeout(&node_115);
    detail::clearStaleTimeout(&node_116);
    detail::clearStaleTimeout(&node_117);
    detail::clearStaleTimeout(&node_118);
    detail::clearStaleTimeout(&node_119);
    detail::clearStaleTimeout(&node_120);
    detail::clearStaleTimeout(&node_121);
    detail::clearStaleTimeout(&node_205);
    detail::clearStaleTimeout(&node_233);
    detail::clearStaleTimeout(&node_234);
    detail::clearStaleTimeout(&node_235);
    detail::clearStaleTimeout(&node_236);
    detail::clearStaleTimeout(&node_237);
    detail::clearStaleTimeout(&node_238);
    detail::clearStaleTimeout(&node_239);
    detail::clearStaleTimeout(&node_240);
    detail::clearStaleTimeout(&node_241);
    detail::clearStaleTimeout(&node_242);
    detail::clearStaleTimeout(&node_243);
    detail::clearStaleTimeout(&node_244);
    detail::clearStaleTimeout(&node_245);
    detail::clearStaleTimeout(&node_246);
    detail::clearStaleTimeout(&node_247);
    detail::clearStaleTimeout(&node_248);
    detail::clearStaleTimeout(&node_249);
    detail::clearStaleTimeout(&node_250);
    detail::clearStaleTimeout(&node_251);
    detail::clearStaleTimeout(&node_252);
    detail::clearStaleTimeout(&node_253);
    detail::clearStaleTimeout(&node_254);
    detail::clearStaleTimeout(&node_255);
    detail::clearStaleTimeout(&node_256);
    detail::clearStaleTimeout(&node_257);
    detail::clearStaleTimeout(&node_258);
    detail::clearStaleTimeout(&node_259);
    detail::clearStaleTimeout(&node_260);

    XOD_TRACE_F("Transaction completed, t=");
    XOD_TRACE_LN(millis());
}

} // namespace xod
