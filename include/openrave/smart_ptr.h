#ifndef OPENRAVE_SMART_PTR_H
#define OPENRAVE_SMART_PTR_H

#define OPENRAVE_UNIQUE_PTR boost::unique_ptr
#define OPENRAVE_SHARED_PTR std::shared_ptr
#define OPENRAVE_WEAK_PTR std::weak_ptr
#define OPENRAVE_STATIC_POINTER_CAST std::static_pointer_cast
#define OPENRAVE_ENABLE_SHARED_FROM_THIS std::enable_shared_from_this
#define OPENRAVE_DYNAMIC_POINTER_CAST std::dynamic_pointer_cast
#define OPENRAVE_CONST_POINTER_CAST std::const_pointer_cast
#define OPENRAVE_MAKE_SHARED std::make_shared
// std::function does not have "clear" method
#define OPENRAVE_FUNCTION std::function

#endif // OPENRAVE_SMART_PTR_H