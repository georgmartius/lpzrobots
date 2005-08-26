#ifndef exception_h
#define exception_h

#define EXCEPTION_TEMPLATE(class_name)                                        \
  class class_name : public IException {                                      \
   public:                                                                    \
    inline virtual void raise() const { throw *this; }                        \
  };


namespace university_of_leipzig {
namespace robot {
namespace exception {


class IException {
 public:
  virtual void raise() const = 0;
};


EXCEPTION_TEMPLATE(IndexOutOfBounds);
EXCEPTION_TEMPLATE(InvalidArgument);
EXCEPTION_TEMPLATE(DimensionMismatch);



}
}
}

#endif
