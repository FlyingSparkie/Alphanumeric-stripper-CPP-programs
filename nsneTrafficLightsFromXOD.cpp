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
// xod/core/continuously implementation
//-----------------------------------------------------------------------------
namespace xod__core__continuously {

struct State {
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

struct output_TICK { };

template<typename PinT> struct ValueType { using T = void; };
template<> struct ValueType<output_TICK> { using T = Logic; };

struct ContextObject {
    Node* _node;

};

using Context = ContextObject*;

template<typename PinT> typename ValueType<PinT>::T getValue(Context ctx) {
    static_assert(always_false<PinT>::value,
            "Invalid pin descriptor. Expected one of:" \
            "" \
            " output_TICK");
}

template<> Logic getValue<output_TICK>(Context ctx) {
    return ctx->_node->output_TICK;
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
    emitValue<output_TICK>(ctx, 1);
    setTimeout(ctx, 0);
}

} // namespace xod__core__continuously

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
// xod/core/digital-input implementation
//-----------------------------------------------------------------------------
namespace xod__core__digital_input {

struct State {
    int configuredPort = -1;
};

struct Node {
    State state;
    Logic output_SIG;

    union {
        struct {
            bool isOutputDirty_SIG : 1;
            bool isNodeDirty : 1;
        };

        DirtyFlags dirtyFlags;
    };
};

struct input_PORT { };
struct input_UPD { };
struct output_SIG { };

template<typename PinT> struct ValueType { using T = void; };
template<> struct ValueType<input_PORT> { using T = Number; };
template<> struct ValueType<input_UPD> { using T = Logic; };
template<> struct ValueType<output_SIG> { using T = Logic; };

struct ContextObject {
    Node* _node;

    Number _input_PORT;
    Logic _input_UPD;

    bool _isInputDirty_UPD;
};

using Context = ContextObject*;

template<typename PinT> typename ValueType<PinT>::T getValue(Context ctx) {
    static_assert(always_false<PinT>::value,
            "Invalid pin descriptor. Expected one of:" \
            " input_PORT input_UPD" \
            " output_SIG");
}

template<> Number getValue<input_PORT>(Context ctx) {
    return ctx->_input_PORT;
}
template<> Logic getValue<input_UPD>(Context ctx) {
    return ctx->_input_UPD;
}
template<> Logic getValue<output_SIG>(Context ctx) {
    return ctx->_node->output_SIG;
}

template<typename InputT> bool isInputDirty(Context ctx) {
    static_assert(always_false<InputT>::value,
            "Invalid input descriptor. Expected one of:" \
            " input_UPD");
    return false;
}

template<> bool isInputDirty<input_UPD>(Context ctx) {
    return ctx->_isInputDirty_UPD;
}

template<typename OutputT> void emitValue(Context ctx, typename ValueType<OutputT>::T val) {
    static_assert(always_false<OutputT>::value,
            "Invalid output descriptor. Expected one of:" \
            " output_SIG");
}

template<> void emitValue<output_SIG>(Context ctx, Logic val) {
    ctx->_node->output_SIG = val;
    ctx->_node->isOutputDirty_SIG = true;
}

State* getState(Context ctx) {
    return &ctx->_node->state;
}

void evaluate(Context ctx) {
    if (!isInputDirty<input_UPD>(ctx))
        return;

    State* state = getState(ctx);
    const int port = (int)getValue<input_PORT>(ctx);
    if (port != state->configuredPort) {
        ::pinMode(port, INPUT);
        // Store configured port so to avoid repeating `pinMode` on
        // subsequent requests
        state->configuredPort = port;
    }

    emitValue<output_SIG>(ctx, ::digitalRead(port));
}

} // namespace xod__core__digital_input

//-----------------------------------------------------------------------------
// xod/core/not implementation
//-----------------------------------------------------------------------------
namespace xod__core__not {

#pragma XOD dirtieness disable

struct State {
};

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
}

State* getState(Context ctx) {
    return &ctx->_node->state;
}

void evaluate(Context ctx) {
    auto x = getValue<input_IN>(ctx);
    emitValue<output_OUT>(ctx, !x);
}

} // namespace xod__core__not

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
// xod/core/or implementation
//-----------------------------------------------------------------------------
namespace xod__core__or {

#pragma XOD dirtieness disable

struct State {
};

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
template<> struct ValueType<input_IN1> { using T = Logic; };
template<> struct ValueType<input_IN2> { using T = Logic; };
template<> struct ValueType<output_OUT> { using T = Logic; };

struct ContextObject {
    Node* _node;

    Logic _input_IN1;
    Logic _input_IN2;

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
    auto a = getValue<input_IN1>(ctx);
    auto b = getValue<input_IN2>(ctx);
    emitValue<output_OUT>(ctx, a || b);
}

} // namespace xod__core__or

//-----------------------------------------------------------------------------
// xod/core/branch implementation
//-----------------------------------------------------------------------------
namespace xod__core__branch {

struct State {
};

struct Node {
    State state;
    Logic output_T;
    Logic output_F;

    union {
        struct {
            bool isOutputDirty_T : 1;
            bool isOutputDirty_F : 1;
            bool isNodeDirty : 1;
        };

        DirtyFlags dirtyFlags;
    };
};

struct input_GATE { };
struct input_TRIG { };
struct output_T { };
struct output_F { };

template<typename PinT> struct ValueType { using T = void; };
template<> struct ValueType<input_GATE> { using T = Logic; };
template<> struct ValueType<input_TRIG> { using T = Logic; };
template<> struct ValueType<output_T> { using T = Logic; };
template<> struct ValueType<output_F> { using T = Logic; };

struct ContextObject {
    Node* _node;

    Logic _input_GATE;
    Logic _input_TRIG;

    bool _isInputDirty_TRIG;
};

using Context = ContextObject*;

template<typename PinT> typename ValueType<PinT>::T getValue(Context ctx) {
    static_assert(always_false<PinT>::value,
            "Invalid pin descriptor. Expected one of:" \
            " input_GATE input_TRIG" \
            " output_T output_F");
}

template<> Logic getValue<input_GATE>(Context ctx) {
    return ctx->_input_GATE;
}
template<> Logic getValue<input_TRIG>(Context ctx) {
    return ctx->_input_TRIG;
}
template<> Logic getValue<output_T>(Context ctx) {
    return ctx->_node->output_T;
}
template<> Logic getValue<output_F>(Context ctx) {
    return ctx->_node->output_F;
}

template<typename InputT> bool isInputDirty(Context ctx) {
    static_assert(always_false<InputT>::value,
            "Invalid input descriptor. Expected one of:" \
            " input_TRIG");
    return false;
}

template<> bool isInputDirty<input_TRIG>(Context ctx) {
    return ctx->_isInputDirty_TRIG;
}

template<typename OutputT> void emitValue(Context ctx, typename ValueType<OutputT>::T val) {
    static_assert(always_false<OutputT>::value,
            "Invalid output descriptor. Expected one of:" \
            " output_T output_F");
}

template<> void emitValue<output_T>(Context ctx, Logic val) {
    ctx->_node->output_T = val;
    ctx->_node->isOutputDirty_T = true;
}
template<> void emitValue<output_F>(Context ctx, Logic val) {
    ctx->_node->output_F = val;
    ctx->_node->isOutputDirty_F = true;
}

State* getState(Context ctx) {
    return &ctx->_node->state;
}

void evaluate(Context ctx) {
    if (!isInputDirty<input_TRIG>(ctx))
        return;

    if (getValue<input_GATE>(ctx)) {
        emitValue<output_T>(ctx, 1);
    } else {
        emitValue<output_F>(ctx, 1);
    }
}

} // namespace xod__core__branch

//-----------------------------------------------------------------------------
// xod/core/and implementation
//-----------------------------------------------------------------------------
namespace xod__core__and {

#pragma XOD dirtieness disable

struct State {
};

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
template<> struct ValueType<input_IN1> { using T = Logic; };
template<> struct ValueType<input_IN2> { using T = Logic; };
template<> struct ValueType<output_OUT> { using T = Logic; };

struct ContextObject {
    Node* _node;

    Logic _input_IN1;
    Logic _input_IN2;

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
    auto a = getValue<input_IN1>(ctx);
    auto b = getValue<input_IN2>(ctx);
    emitValue<output_OUT>(ctx, a && b);
}

} // namespace xod__core__and

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
// xod/core/flip-flop implementation
//-----------------------------------------------------------------------------
namespace xod__core__flip_flop {

struct State {
};

struct Node {
    State state;
    Logic output_MEM;

    union {
        struct {
            bool isOutputDirty_MEM : 1;
            bool isNodeDirty : 1;
        };

        DirtyFlags dirtyFlags;
    };
};

struct input_SET { };
struct input_TGL { };
struct input_RST { };
struct output_MEM { };

template<typename PinT> struct ValueType { using T = void; };
template<> struct ValueType<input_SET> { using T = Logic; };
template<> struct ValueType<input_TGL> { using T = Logic; };
template<> struct ValueType<input_RST> { using T = Logic; };
template<> struct ValueType<output_MEM> { using T = Logic; };

struct ContextObject {
    Node* _node;

    Logic _input_SET;
    Logic _input_TGL;
    Logic _input_RST;

    bool _isInputDirty_SET;
    bool _isInputDirty_TGL;
    bool _isInputDirty_RST;
};

using Context = ContextObject*;

template<typename PinT> typename ValueType<PinT>::T getValue(Context ctx) {
    static_assert(always_false<PinT>::value,
            "Invalid pin descriptor. Expected one of:" \
            " input_SET input_TGL input_RST" \
            " output_MEM");
}

template<> Logic getValue<input_SET>(Context ctx) {
    return ctx->_input_SET;
}
template<> Logic getValue<input_TGL>(Context ctx) {
    return ctx->_input_TGL;
}
template<> Logic getValue<input_RST>(Context ctx) {
    return ctx->_input_RST;
}
template<> Logic getValue<output_MEM>(Context ctx) {
    return ctx->_node->output_MEM;
}

template<typename InputT> bool isInputDirty(Context ctx) {
    static_assert(always_false<InputT>::value,
            "Invalid input descriptor. Expected one of:" \
            " input_SET input_TGL input_RST");
    return false;
}

template<> bool isInputDirty<input_SET>(Context ctx) {
    return ctx->_isInputDirty_SET;
}
template<> bool isInputDirty<input_TGL>(Context ctx) {
    return ctx->_isInputDirty_TGL;
}
template<> bool isInputDirty<input_RST>(Context ctx) {
    return ctx->_isInputDirty_RST;
}

template<typename OutputT> void emitValue(Context ctx, typename ValueType<OutputT>::T val) {
    static_assert(always_false<OutputT>::value,
            "Invalid output descriptor. Expected one of:" \
            " output_MEM");
}

template<> void emitValue<output_MEM>(Context ctx, Logic val) {
    ctx->_node->output_MEM = val;
    ctx->_node->isOutputDirty_MEM = true;
}

State* getState(Context ctx) {
    return &ctx->_node->state;
}

void evaluate(Context ctx) {
    bool oldState = getValue<output_MEM>(ctx);
    bool newState = oldState;

    if (isInputDirty<input_TGL>(ctx)) {
        newState = !oldState;
    } else if (isInputDirty<input_SET>(ctx)) {
        newState = true;
    } else {
        newState = false;
    }

    if (newState == oldState)
        return;

    emitValue<output_MEM>(ctx, newState);
}

} // namespace xod__core__flip_flop

//-----------------------------------------------------------------------------
// xod/core/digital-output implementation
//-----------------------------------------------------------------------------
namespace xod__core__digital_output {

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
struct input_SIG { };

template<typename PinT> struct ValueType { using T = void; };
template<> struct ValueType<input_PORT> { using T = Number; };
template<> struct ValueType<input_SIG> { using T = Logic; };

struct ContextObject {
    Node* _node;

    Number _input_PORT;
    Logic _input_SIG;

};

using Context = ContextObject*;

template<typename PinT> typename ValueType<PinT>::T getValue(Context ctx) {
    static_assert(always_false<PinT>::value,
            "Invalid pin descriptor. Expected one of:" \
            " input_PORT input_SIG" \
            "");
}

template<> Number getValue<input_PORT>(Context ctx) {
    return ctx->_input_PORT;
}
template<> Logic getValue<input_SIG>(Context ctx) {
    return ctx->_input_SIG;
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

    const bool val = getValue<input_SIG>(ctx);
    ::digitalWrite(port, val);
}

} // namespace xod__core__digital_output

//-----------------------------------------------------------------------------
// xod/core/nor implementation
//-----------------------------------------------------------------------------
namespace xod__core__nor {

#pragma XOD dirtieness disable

struct State {
};

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
template<> struct ValueType<input_IN1> { using T = Logic; };
template<> struct ValueType<input_IN2> { using T = Logic; };
template<> struct ValueType<output_OUT> { using T = Logic; };

struct ContextObject {
    Node* _node;

    Logic _input_IN1;
    Logic _input_IN2;

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
    auto a = getValue<input_IN1>(ctx);
    auto b = getValue<input_IN2>(ctx);
    emitValue<output_OUT>(ctx, !(a || b));
}

} // namespace xod__core__nor

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

constexpr Number node_1_output_VAL = 2;

constexpr Number node_2_output_VAL = 3;

constexpr Number node_3_output_VAL = 4;

constexpr Number node_4_output_VAL = 2;

constexpr Number node_5_output_VAL = 1;

constexpr Number node_6_output_VAL = 12;

constexpr Number node_7_output_VAL = 6;

constexpr Number node_8_output_VAL = 7;

constexpr Number node_9_output_VAL = 5;

constexpr Number node_10_output_VAL = 1;

constexpr Number node_11_output_VAL = 2;

constexpr Number node_12_output_VAL = 11;

constexpr Number node_13_output_VAL = 2;

constexpr Number node_14_output_VAL = 9;

constexpr Number node_15_output_VAL = 10;

constexpr Number node_16_output_VAL = 8;

constexpr Number node_17_output_VAL = 1;

constexpr Logic node_18_output_TICK = false;

constexpr Logic node_19_output_DONE = false;
constexpr Logic node_19_output_ACT = false;

constexpr Logic node_20_output_SIG = false;

constexpr Logic node_21_output_SIG = false;

constexpr Logic node_22_output_OUT = false;

constexpr Logic node_23_output_OUT = false;

constexpr Logic node_24_output_OUT = false;

constexpr Logic node_25_output_T = false;
constexpr Logic node_25_output_F = false;

constexpr Logic node_26_output_OUT = false;

constexpr Logic node_27_output_OUT = false;

constexpr Logic node_28_output_DONE = false;
constexpr Logic node_28_output_ACT = false;

constexpr Logic node_29_output_OUT = false;

constexpr Logic node_30_output_T = false;
constexpr Logic node_30_output_F = false;

constexpr Logic node_31_output_DONE = false;
constexpr Logic node_31_output_ACT = false;

constexpr Logic node_32_output_OUT = false;

constexpr Logic node_33_output_MEM = false;

constexpr Logic node_34_output_DONE = false;
constexpr Logic node_34_output_ACT = false;

constexpr Logic node_35_output_OUT = false;

constexpr Logic node_36_output_DONE = false;
constexpr Logic node_36_output_ACT = false;

constexpr Logic node_38_output_MEM = false;

constexpr Logic node_41_output_OUT = false;

constexpr Logic node_42_output_OUT = false;

constexpr Logic node_43_output_DONE = false;
constexpr Logic node_43_output_ACT = false;

constexpr Logic node_44_output_OUT = false;

constexpr Logic node_46_output_OUT = false;

constexpr Logic node_49_output_MEM = false;

constexpr Logic node_52_output_OUT = false;

constexpr Logic node_54_output_OUT = false;

constexpr Logic node_55_output_OUT = false;

constexpr Logic node_56_output_OUT = false;

constexpr Logic node_57_output_OUT = false;

#pragma GCC diagnostic pop

xod__core__boot::Node node_0 = {
    xod__core__boot::State(), // state default
    node_0_output_BOOT, // output BOOT default
    false, // BOOT dirty
    true // node itself dirty
};
xod__core__continuously::Node node_18 = {
    xod__core__continuously::State(), // state default
    0, // timeoutAt
    node_18_output_TICK, // output TICK default
    false, // TICK dirty
    true // node itself dirty
};

xod__core__delay::Node node_19 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_19_output_DONE, // output DONE default
    node_19_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__digital_input::Node node_20 = {
    xod__core__digital_input::State(), // state default
    node_20_output_SIG, // output SIG default
    true, // SIG dirty
    true // node itself dirty
};
xod__core__digital_input::Node node_21 = {
    xod__core__digital_input::State(), // state default
    node_21_output_SIG, // output SIG default
    true, // SIG dirty
    true // node itself dirty
};
xod__core__not::Node node_22 = {
    xod__core__not::State(), // state default
    node_22_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__cast_to_pulse__boolean::Node node_23 = {
    xod__core__cast_to_pulse__boolean::State(), // state default
    node_23_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__or::Node node_24 = {
    xod__core__or::State(), // state default
    node_24_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__branch::Node node_25 = {
    xod__core__branch::State(), // state default
    node_25_output_T, // output T default
    node_25_output_F, // output F default
    false, // T dirty
    false, // F dirty
    true // node itself dirty
};
xod__core__and::Node node_26 = {
    xod__core__and::State(), // state default
    node_26_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__any::Node node_27 = {
    xod__core__any::State(), // state default
    node_27_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__delay::Node node_28 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_28_output_DONE, // output DONE default
    node_28_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__cast_to_pulse__boolean::Node node_29 = {
    xod__core__cast_to_pulse__boolean::State(), // state default
    node_29_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__branch::Node node_30 = {
    xod__core__branch::State(), // state default
    node_30_output_T, // output T default
    node_30_output_F, // output F default
    false, // T dirty
    false, // F dirty
    true // node itself dirty
};
xod__core__delay::Node node_31 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_31_output_DONE, // output DONE default
    node_31_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__cast_to_pulse__boolean::Node node_32 = {
    xod__core__cast_to_pulse__boolean::State(), // state default
    node_32_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__flip_flop::Node node_33 = {
    xod__core__flip_flop::State(), // state default
    node_33_output_MEM, // output MEM default
    true, // MEM dirty
    true // node itself dirty
};
xod__core__delay::Node node_34 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_34_output_DONE, // output DONE default
    node_34_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__any::Node node_35 = {
    xod__core__any::State(), // state default
    node_35_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__delay::Node node_36 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_36_output_DONE, // output DONE default
    node_36_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__digital_output::Node node_37 = {
    xod__core__digital_output::State(), // state default
    true // node itself dirty
};
xod__core__flip_flop::Node node_38 = {
    xod__core__flip_flop::State(), // state default
    node_38_output_MEM, // output MEM default
    true, // MEM dirty
    true // node itself dirty
};
xod__core__digital_output::Node node_39 = {
    xod__core__digital_output::State(), // state default
    true // node itself dirty
};
xod__core__digital_output::Node node_40 = {
    xod__core__digital_output::State(), // state default
    true // node itself dirty
};
xod__core__nor::Node node_41 = {
    xod__core__nor::State(), // state default
    node_41_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__any::Node node_42 = {
    xod__core__any::State(), // state default
    node_42_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__delay::Node node_43 = {
    xod__core__delay::State(), // state default
    0, // timeoutAt
    node_43_output_DONE, // output DONE default
    node_43_output_ACT, // output ACT default
    false, // DONE dirty
    true, // ACT dirty
    true // node itself dirty
};
xod__core__cast_to_pulse__boolean::Node node_44 = {
    xod__core__cast_to_pulse__boolean::State(), // state default
    node_44_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__digital_output::Node node_45 = {
    xod__core__digital_output::State(), // state default
    true // node itself dirty
};
xod__core__nor::Node node_46 = {
    xod__core__nor::State(), // state default
    node_46_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__digital_output::Node node_47 = {
    xod__core__digital_output::State(), // state default
    true // node itself dirty
};
xod__core__digital_output::Node node_48 = {
    xod__core__digital_output::State(), // state default
    true // node itself dirty
};
xod__core__flip_flop::Node node_49 = {
    xod__core__flip_flop::State(), // state default
    node_49_output_MEM, // output MEM default
    true, // MEM dirty
    true // node itself dirty
};
xod__core__digital_output::Node node_50 = {
    xod__core__digital_output::State(), // state default
    true // node itself dirty
};
xod__core__digital_output::Node node_51 = {
    xod__core__digital_output::State(), // state default
    true // node itself dirty
};
xod__core__nor::Node node_52 = {
    xod__core__nor::State(), // state default
    node_52_output_OUT, // output OUT default
    true // node itself dirty
};
xod__core__digital_output::Node node_53 = {
    xod__core__digital_output::State(), // state default
    true // node itself dirty
};
xod__core__defer__pulse::Node node_54 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_54_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_55 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_55_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_56 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_56_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};
xod__core__defer__pulse::Node node_57 = {
    xod__core__defer__pulse::State(), // state default
    0, // timeoutAt
    node_57_output_OUT, // output OUT default
    false, // OUT dirty
    true // node itself dirty
};

void runTransaction() {
    g_transactionTime = millis();

    XOD_TRACE_F("Transaction started, t=");
    XOD_TRACE_LN(g_transactionTime);

    // Check for timeouts
    detail::checkTriggerTimeout(&node_18);
    detail::checkTriggerTimeout(&node_19);
    detail::checkTriggerTimeout(&node_28);
    detail::checkTriggerTimeout(&node_31);
    detail::checkTriggerTimeout(&node_34);
    detail::checkTriggerTimeout(&node_36);
    detail::checkTriggerTimeout(&node_43);
    detail::checkTriggerTimeout(&node_54);
    detail::checkTriggerTimeout(&node_55);
    detail::checkTriggerTimeout(&node_56);
    detail::checkTriggerTimeout(&node_57);

    // defer-* nodes are always at the very bottom of the graph, so no one will
    // recieve values emitted by them. We must evaluate them before everybody
    // else to give them a chance to emit values.
    //
    // If trigerred, keep only output dirty, not the node itself, so it will
    // evaluate on the regular pass only if it pushed a new value again.
    {
        if (node_54.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(54);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_54;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_27.isNodeDirty |= node_54.isOutputDirty_OUT;

            node_54.isNodeDirty = false;
            detail::clearTimeout(&node_54);
        }
    }
    {
        if (node_55.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(55);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_55;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_25.isNodeDirty |= node_55.isOutputDirty_OUT;

            node_55.isNodeDirty = false;
            detail::clearTimeout(&node_55);
        }
    }
    {
        if (node_56.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(56);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_56;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_19.isNodeDirty |= node_56.isOutputDirty_OUT;

            node_56.isNodeDirty = false;
            detail::clearTimeout(&node_56);
        }
    }
    {
        if (node_57.isNodeDirty) {
            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(57);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_57;
            ctxObj._isInputDirty_IN = false;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_35.isNodeDirty |= node_57.isOutputDirty_OUT;

            node_57.isNodeDirty = false;
            detail::clearTimeout(&node_57);
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
            node_42.isNodeDirty |= node_0.isOutputDirty_BOOT;
        }
    }
    { // xod__core__continuously #18
        if (node_18.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(18);

            xod__core__continuously::ContextObject ctxObj;
            ctxObj._node = &node_18;

            // copy data from upstream nodes into context

            xod__core__continuously::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_20.isNodeDirty |= node_18.isOutputDirty_TICK;
            node_21.isNodeDirty |= node_18.isOutputDirty_TICK;
        }
    }
    { // xod__core__delay #19
        if (node_19.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(19);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_19;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_1_output_VAL;
            ctxObj._input_SET = node_56.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_56.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_23.isNodeDirty |= node_19.isOutputDirty_ACT;
            node_22.isNodeDirty |= node_19.isOutputDirty_ACT;
        }
    }
    { // xod__core__digital_input #20
        if (node_20.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(20);

            xod__core__digital_input::ContextObject ctxObj;
            ctxObj._node = &node_20;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_6_output_VAL;
            ctxObj._input_UPD = node_18.output_TICK;

            ctxObj._isInputDirty_UPD = node_18.isOutputDirty_TICK;

            xod__core__digital_input::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_24.isNodeDirty |= node_20.isOutputDirty_SIG;
            node_30.isNodeDirty |= node_20.isOutputDirty_SIG;
        }
    }
    { // xod__core__digital_input #21
        if (node_21.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(21);

            xod__core__digital_input::ContextObject ctxObj;
            ctxObj._node = &node_21;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_12_output_VAL;
            ctxObj._input_UPD = node_18.output_TICK;

            ctxObj._isInputDirty_UPD = node_18.isOutputDirty_TICK;

            xod__core__digital_input::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_24.isNodeDirty |= node_21.isOutputDirty_SIG;
            node_25.isNodeDirty |= node_21.isOutputDirty_SIG;
        }
    }
    { // xod__core__not #22
        if (node_22.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(22);

            xod__core__not::ContextObject ctxObj;
            ctxObj._node = &node_22;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_19.output_ACT;

            xod__core__not::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_26.isNodeDirty = true;
        }
    }
    { // xod__core__cast_to_pulse__boolean #23
        if (node_23.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(23);

            xod__core__cast_to_pulse__boolean::ContextObject ctxObj;
            ctxObj._node = &node_23;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_19.output_ACT;

            xod__core__cast_to_pulse__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_33.isNodeDirty |= node_23.isOutputDirty_OUT;
        }
    }
    { // xod__core__or #24
        if (node_24.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(24);

            xod__core__or::ContextObject ctxObj;
            ctxObj._node = &node_24;

            // copy data from upstream nodes into context
            ctxObj._input_IN1 = node_21.output_SIG;
            ctxObj._input_IN2 = node_20.output_SIG;

            xod__core__or::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_26.isNodeDirty = true;
        }
    }
    { // xod__core__branch #25
        if (node_25.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(25);

            xod__core__branch::ContextObject ctxObj;
            ctxObj._node = &node_25;

            // copy data from upstream nodes into context
            ctxObj._input_GATE = node_21.output_SIG;
            ctxObj._input_TRIG = node_55.output_OUT;

            ctxObj._isInputDirty_TRIG = node_55.isOutputDirty_OUT;

            xod__core__branch::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_28.isNodeDirty |= node_25.isOutputDirty_T;
            node_27.isNodeDirty |= node_25.isOutputDirty_F;
        }
    }
    { // xod__core__and #26
        if (node_26.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(26);

            xod__core__and::ContextObject ctxObj;
            ctxObj._node = &node_26;

            // copy data from upstream nodes into context
            ctxObj._input_IN1 = node_22.output_OUT;
            ctxObj._input_IN2 = node_24.output_OUT;

            xod__core__and::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_29.isNodeDirty = true;
        }
    }
    { // xod__core__any #27
        if (node_27.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(27);

            xod__core__any::ContextObject ctxObj;
            ctxObj._node = &node_27;

            // copy data from upstream nodes into context
            ctxObj._input_IN1 = node_54.output_OUT;
            ctxObj._input_IN2 = node_25.output_F;

            ctxObj._isInputDirty_IN1 = node_54.isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = node_25.isOutputDirty_F;

            xod__core__any::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_30.isNodeDirty |= node_27.isOutputDirty_OUT;
        }
    }
    { // xod__core__delay #28
        if (node_28.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(28);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_28;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_13_output_VAL;
            ctxObj._input_SET = node_25.output_T;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_25.isOutputDirty_T;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_31.isNodeDirty |= node_28.isOutputDirty_DONE;
            node_38.isNodeDirty |= node_28.isOutputDirty_DONE;
            node_32.isNodeDirty |= node_28.isOutputDirty_ACT;
        }
    }
    { // xod__core__cast_to_pulse__boolean #29
        if (node_29.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(29);

            xod__core__cast_to_pulse__boolean::ContextObject ctxObj;
            ctxObj._node = &node_29;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_26.output_OUT;

            xod__core__cast_to_pulse__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_34.isNodeDirty |= node_29.isOutputDirty_OUT;
            node_33.isNodeDirty |= node_29.isOutputDirty_OUT;
        }
    }
    { // xod__core__branch #30
        if (node_30.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(30);

            xod__core__branch::ContextObject ctxObj;
            ctxObj._node = &node_30;

            // copy data from upstream nodes into context
            ctxObj._input_GATE = node_20.output_SIG;
            ctxObj._input_TRIG = node_27.output_OUT;

            ctxObj._isInputDirty_TRIG = node_27.isOutputDirty_OUT;

            xod__core__branch::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_36.isNodeDirty |= node_30.isOutputDirty_T;
            node_35.isNodeDirty |= node_30.isOutputDirty_F;
        }
    }
    { // xod__core__delay #31
        if (node_31.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(31);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_31;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_5_output_VAL;
            ctxObj._input_SET = node_28.output_DONE;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_28.isOutputDirty_DONE;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_54.isNodeDirty |= node_31.isOutputDirty_DONE;
            node_37.isNodeDirty |= node_31.isOutputDirty_ACT;
            node_46.isNodeDirty |= node_31.isOutputDirty_ACT;
        }
    }
    { // xod__core__cast_to_pulse__boolean #32
        if (node_32.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(32);

            xod__core__cast_to_pulse__boolean::ContextObject ctxObj;
            ctxObj._node = &node_32;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_28.output_ACT;

            xod__core__cast_to_pulse__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_38.isNodeDirty |= node_32.isOutputDirty_OUT;
        }
    }
    { // xod__core__flip_flop #33
        if (node_33.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(33);

            xod__core__flip_flop::ContextObject ctxObj;
            ctxObj._node = &node_33;

            // copy data from upstream nodes into context
            ctxObj._input_SET = node_23.output_OUT;
            ctxObj._input_RST = node_29.output_OUT;

            ctxObj._isInputDirty_TGL = false;
            ctxObj._isInputDirty_SET = node_23.isOutputDirty_OUT;
            ctxObj._isInputDirty_RST = node_29.isOutputDirty_OUT;

            xod__core__flip_flop::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_39.isNodeDirty |= node_33.isOutputDirty_MEM;
            node_41.isNodeDirty |= node_33.isOutputDirty_MEM;
        }
    }
    { // xod__core__delay #34
        if (node_34.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(34);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_34;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_10_output_VAL;
            ctxObj._input_SET = node_29.output_OUT;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_29.isOutputDirty_OUT;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_55.isNodeDirty |= node_34.isOutputDirty_DONE;
            node_40.isNodeDirty |= node_34.isOutputDirty_ACT;
            node_41.isNodeDirty |= node_34.isOutputDirty_ACT;
        }
    }
    { // xod__core__any #35
        if (node_35.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(35);

            xod__core__any::ContextObject ctxObj;
            ctxObj._node = &node_35;

            // copy data from upstream nodes into context
            ctxObj._input_IN1 = node_30.output_F;
            ctxObj._input_IN2 = node_57.output_OUT;

            ctxObj._isInputDirty_IN1 = node_30.isOutputDirty_F;
            ctxObj._isInputDirty_IN2 = node_57.isOutputDirty_OUT;

            xod__core__any::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_42.isNodeDirty |= node_35.isOutputDirty_OUT;
        }
    }
    { // xod__core__delay #36
        if (node_36.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(36);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_36;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_11_output_VAL;
            ctxObj._input_SET = node_30.output_T;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_30.isOutputDirty_T;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_43.isNodeDirty |= node_36.isOutputDirty_DONE;
            node_49.isNodeDirty |= node_36.isOutputDirty_DONE;
            node_44.isNodeDirty |= node_36.isOutputDirty_ACT;
        }
    }
    { // xod__core__digital_output #37
        if (node_37.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(37);

            xod__core__digital_output::ContextObject ctxObj;
            ctxObj._node = &node_37;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_2_output_VAL;
            ctxObj._input_SIG = node_31.output_ACT;

            xod__core__digital_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__flip_flop #38
        if (node_38.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(38);

            xod__core__flip_flop::ContextObject ctxObj;
            ctxObj._node = &node_38;

            // copy data from upstream nodes into context
            ctxObj._input_SET = node_32.output_OUT;
            ctxObj._input_RST = node_28.output_DONE;

            ctxObj._isInputDirty_TGL = false;
            ctxObj._isInputDirty_SET = node_32.isOutputDirty_OUT;
            ctxObj._isInputDirty_RST = node_28.isOutputDirty_DONE;

            xod__core__flip_flop::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_45.isNodeDirty |= node_38.isOutputDirty_MEM;
            node_46.isNodeDirty |= node_38.isOutputDirty_MEM;
        }
    }
    { // xod__core__digital_output #39
        if (node_39.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(39);

            xod__core__digital_output::ContextObject ctxObj;
            ctxObj._node = &node_39;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_9_output_VAL;
            ctxObj._input_SIG = node_33.output_MEM;

            xod__core__digital_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__digital_output #40
        if (node_40.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(40);

            xod__core__digital_output::ContextObject ctxObj;
            ctxObj._node = &node_40;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_7_output_VAL;
            ctxObj._input_SIG = node_34.output_ACT;

            xod__core__digital_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__nor #41
        if (node_41.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(41);

            xod__core__nor::ContextObject ctxObj;
            ctxObj._node = &node_41;

            // copy data from upstream nodes into context
            ctxObj._input_IN1 = node_33.output_MEM;
            ctxObj._input_IN2 = node_34.output_ACT;

            xod__core__nor::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_47.isNodeDirty = true;
        }
    }
    { // xod__core__any #42
        if (node_42.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(42);

            xod__core__any::ContextObject ctxObj;
            ctxObj._node = &node_42;

            // copy data from upstream nodes into context
            ctxObj._input_IN1 = node_0.output_BOOT;
            ctxObj._input_IN2 = node_35.output_OUT;

            ctxObj._isInputDirty_IN1 = node_0.isOutputDirty_BOOT;
            ctxObj._isInputDirty_IN2 = node_35.isOutputDirty_OUT;

            xod__core__any::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_56.isNodeDirty |= node_42.isOutputDirty_OUT;
        }
    }
    { // xod__core__delay #43
        if (node_43.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(43);

            xod__core__delay::ContextObject ctxObj;
            ctxObj._node = &node_43;

            // copy data from upstream nodes into context
            ctxObj._input_T = node_17_output_VAL;
            ctxObj._input_SET = node_36.output_DONE;

            ctxObj._isInputDirty_RST = false;
            ctxObj._isInputDirty_SET = node_36.isOutputDirty_DONE;

            xod__core__delay::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_57.isNodeDirty |= node_43.isOutputDirty_DONE;
            node_48.isNodeDirty |= node_43.isOutputDirty_ACT;
            node_52.isNodeDirty |= node_43.isOutputDirty_ACT;
        }
    }
    { // xod__core__cast_to_pulse__boolean #44
        if (node_44.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(44);

            xod__core__cast_to_pulse__boolean::ContextObject ctxObj;
            ctxObj._node = &node_44;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_36.output_ACT;

            xod__core__cast_to_pulse__boolean::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_49.isNodeDirty |= node_44.isOutputDirty_OUT;
        }
    }
    { // xod__core__digital_output #45
        if (node_45.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(45);

            xod__core__digital_output::ContextObject ctxObj;
            ctxObj._node = &node_45;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_4_output_VAL;
            ctxObj._input_SIG = node_38.output_MEM;

            xod__core__digital_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__nor #46
        if (node_46.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(46);

            xod__core__nor::ContextObject ctxObj;
            ctxObj._node = &node_46;

            // copy data from upstream nodes into context
            ctxObj._input_IN1 = node_38.output_MEM;
            ctxObj._input_IN2 = node_31.output_ACT;

            xod__core__nor::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_50.isNodeDirty = true;
        }
    }
    { // xod__core__digital_output #47
        if (node_47.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(47);

            xod__core__digital_output::ContextObject ctxObj;
            ctxObj._node = &node_47;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_8_output_VAL;
            ctxObj._input_SIG = node_41.output_OUT;

            xod__core__digital_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__digital_output #48
        if (node_48.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(48);

            xod__core__digital_output::ContextObject ctxObj;
            ctxObj._node = &node_48;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_14_output_VAL;
            ctxObj._input_SIG = node_43.output_ACT;

            xod__core__digital_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__flip_flop #49
        if (node_49.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(49);

            xod__core__flip_flop::ContextObject ctxObj;
            ctxObj._node = &node_49;

            // copy data from upstream nodes into context
            ctxObj._input_SET = node_44.output_OUT;
            ctxObj._input_RST = node_36.output_DONE;

            ctxObj._isInputDirty_TGL = false;
            ctxObj._isInputDirty_SET = node_44.isOutputDirty_OUT;
            ctxObj._isInputDirty_RST = node_36.isOutputDirty_DONE;

            xod__core__flip_flop::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_51.isNodeDirty |= node_49.isOutputDirty_MEM;
            node_52.isNodeDirty |= node_49.isOutputDirty_MEM;
        }
    }
    { // xod__core__digital_output #50
        if (node_50.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(50);

            xod__core__digital_output::ContextObject ctxObj;
            ctxObj._node = &node_50;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_3_output_VAL;
            ctxObj._input_SIG = node_46.output_OUT;

            xod__core__digital_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__digital_output #51
        if (node_51.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(51);

            xod__core__digital_output::ContextObject ctxObj;
            ctxObj._node = &node_51;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_16_output_VAL;
            ctxObj._input_SIG = node_49.output_MEM;

            xod__core__digital_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__nor #52
        if (node_52.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(52);

            xod__core__nor::ContextObject ctxObj;
            ctxObj._node = &node_52;

            // copy data from upstream nodes into context
            ctxObj._input_IN1 = node_49.output_MEM;
            ctxObj._input_IN2 = node_43.output_ACT;

            xod__core__nor::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_53.isNodeDirty = true;
        }
    }
    { // xod__core__digital_output #53
        if (node_53.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(53);

            xod__core__digital_output::ContextObject ctxObj;
            ctxObj._node = &node_53;

            // copy data from upstream nodes into context
            ctxObj._input_PORT = node_15_output_VAL;
            ctxObj._input_SIG = node_52.output_OUT;

            xod__core__digital_output::evaluate(&ctxObj);

            // mark downstream nodes dirty
        }
    }
    { // xod__core__defer__pulse #54
        if (node_54.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(54);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_54;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_31.output_DONE;

            ctxObj._isInputDirty_IN = node_31.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_27.isNodeDirty |= node_54.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #55
        if (node_55.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(55);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_55;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_34.output_DONE;

            ctxObj._isInputDirty_IN = node_34.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_25.isNodeDirty |= node_55.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #56
        if (node_56.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(56);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_56;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_42.output_OUT;

            ctxObj._isInputDirty_IN = node_42.isOutputDirty_OUT;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_19.isNodeDirty |= node_56.isOutputDirty_OUT;
        }
    }
    { // xod__core__defer__pulse #57
        if (node_57.isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(57);

            xod__core__defer__pulse::ContextObject ctxObj;
            ctxObj._node = &node_57;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_43.output_DONE;

            ctxObj._isInputDirty_IN = node_43.isOutputDirty_DONE;

            xod__core__defer__pulse::evaluate(&ctxObj);

            // mark downstream nodes dirty
            node_35.isNodeDirty |= node_57.isOutputDirty_OUT;
        }
    }

    // Clear dirtieness and timeouts for all nodes and pins
    node_0.dirtyFlags = 0;
    node_18.dirtyFlags = 0;
    node_19.dirtyFlags = 0;
    node_20.dirtyFlags = 0;
    node_21.dirtyFlags = 0;
    node_22.dirtyFlags = 0;
    node_23.dirtyFlags = 0;
    node_24.dirtyFlags = 0;
    node_25.dirtyFlags = 0;
    node_26.dirtyFlags = 0;
    node_27.dirtyFlags = 0;
    node_28.dirtyFlags = 0;
    node_29.dirtyFlags = 0;
    node_30.dirtyFlags = 0;
    node_31.dirtyFlags = 0;
    node_32.dirtyFlags = 0;
    node_33.dirtyFlags = 0;
    node_34.dirtyFlags = 0;
    node_35.dirtyFlags = 0;
    node_36.dirtyFlags = 0;
    node_37.dirtyFlags = 0;
    node_38.dirtyFlags = 0;
    node_39.dirtyFlags = 0;
    node_40.dirtyFlags = 0;
    node_41.dirtyFlags = 0;
    node_42.dirtyFlags = 0;
    node_43.dirtyFlags = 0;
    node_44.dirtyFlags = 0;
    node_45.dirtyFlags = 0;
    node_46.dirtyFlags = 0;
    node_47.dirtyFlags = 0;
    node_48.dirtyFlags = 0;
    node_49.dirtyFlags = 0;
    node_50.dirtyFlags = 0;
    node_51.dirtyFlags = 0;
    node_52.dirtyFlags = 0;
    node_53.dirtyFlags = 0;
    node_54.dirtyFlags = 0;
    node_55.dirtyFlags = 0;
    node_56.dirtyFlags = 0;
    node_57.dirtyFlags = 0;
    detail::clearStaleTimeout(&node_18);
    detail::clearStaleTimeout(&node_19);
    detail::clearStaleTimeout(&node_28);
    detail::clearStaleTimeout(&node_31);
    detail::clearStaleTimeout(&node_34);
    detail::clearStaleTimeout(&node_36);
    detail::clearStaleTimeout(&node_43);
    detail::clearStaleTimeout(&node_54);
    detail::clearStaleTimeout(&node_55);
    detail::clearStaleTimeout(&node_56);
    detail::clearStaleTimeout(&node_57);

    XOD_TRACE_F("Transaction completed, t=");
    XOD_TRACE_LN(millis());
}

} // namespace xod
