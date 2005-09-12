#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

#define EXCEPTION_TEMPLATE(class_name)                                        \
  class class_name : public IException {                                      \
   public:                                                                    \
    inline virtual void raise() const { throw *this; }                        \
  };


namespace university_of_leipzig {
namespace robots {


class IException {
 public:
  virtual void raise() const = 0;
};


EXCEPTION_TEMPLATE(IndexOutOfBoundsException);
EXCEPTION_TEMPLATE(InvalidArgumentException);
EXCEPTION_TEMPLATE(DimensionMismatchException);



}
}

#endif
