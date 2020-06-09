// **********************************************************************
//
// Copyright (c) 2003-2017 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************
//
// Ice version 3.7.0
//
// <auto-generated>
//
// Generated from file `Laser.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef __Laser_h__
#define __Laser_h__

#include <IceUtil/PushDisableWarnings.h>
#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/ValueF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/StreamHelpers.h>
#include <Ice/Comparable.h>
#include <Ice/Proxy.h>
#include <Ice/Object.h>
#include <Ice/GCObject.h>
#include <Ice/Value.h>
#include <Ice/Incoming.h>
#include <Ice/FactoryTableInit.h>
#include <IceUtil/ScopedArray.h>
#include <Ice/Optional.h>
#include <Ice/ExceptionHelpers.h>
#include <GenericBase.h>
#include <IceUtil/UndefSysMacros.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 307
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 > 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 0
#       error Ice patch level mismatch!
#   endif
#endif

#ifdef ICE_CPP11_MAPPING // C++11 mapping

namespace RoboCompLaser
{

class Laser;
class LaserPrx;

}

namespace RoboCompLaser
{

using shortVector = ::std::vector<int>;

struct LaserConfData
{
    ::std::string driver;
    ::std::string device;
    int staticConf;
    int maxMeasures;
    int maxDegrees;
    int maxRange;
    int minRange;
    int iniRange;
    int endRange;
    int cluster;
    int sampleRate;
    float angleRes;
    float angleIni;

    std::tuple<const ::std::string&, const ::std::string&, const int&, const int&, const int&, const int&, const int&, const int&, const int&, const int&, const int&, const float&, const float&> ice_tuple() const
    {
        return std::tie(driver, device, staticConf, maxMeasures, maxDegrees, maxRange, minRange, iniRange, endRange, cluster, sampleRate, angleRes, angleIni);
    }
};

struct TData
{
    float angle;
    float dist;

    std::tuple<const float&, const float&> ice_tuple() const
    {
        return std::tie(angle, dist);
    }
};

using TLaserData = ::std::vector<::RoboCompLaser::TData>;

using Ice::operator<;
using Ice::operator<=;
using Ice::operator>;
using Ice::operator>=;
using Ice::operator==;
using Ice::operator!=;

}

namespace RoboCompLaser
{

class Laser : public virtual ::Ice::Object
{
public:

    using ProxyType = LaserPrx;

    virtual bool ice_isA(::std::string, const ::Ice::Current&) const override;
    virtual ::std::vector<::std::string> ice_ids(const ::Ice::Current&) const override;
    virtual ::std::string ice_id(const ::Ice::Current&) const override;

    static const ::std::string& ice_staticId();

    struct GetLaserAndBStateDataResult
    {
        ::RoboCompLaser::TLaserData returnValue;
        ::RoboCompGenericBase::TBaseState bState;
    };

    virtual ::RoboCompLaser::TLaserData getLaserAndBStateData(::RoboCompGenericBase::TBaseState&, const ::Ice::Current&) = 0;
    bool _iceD_getLaserAndBStateData(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::RoboCompLaser::LaserConfData getLaserConfData(const ::Ice::Current&) = 0;
    bool _iceD_getLaserConfData(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::RoboCompLaser::TLaserData getLaserData(const ::Ice::Current&) = 0;
    bool _iceD_getLaserData(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual bool _iceDispatch(::IceInternal::Incoming&, const ::Ice::Current&) override;
};

}

namespace RoboCompLaser
{

class LaserPrx : public virtual ::Ice::Proxy<LaserPrx, ::Ice::ObjectPrx>
{
public:

    ::RoboCompLaser::TLaserData getLaserAndBStateData(::RoboCompGenericBase::TBaseState& iceP_bState, const ::Ice::Context& context = Ice::noExplicitContext)
    {
        auto result = _makePromiseOutgoing<::RoboCompLaser::Laser::GetLaserAndBStateDataResult>(true, this, &RoboCompLaser::LaserPrx::_iceI_getLaserAndBStateData, context).get();
        iceP_bState = ::std::move(result.bState);
        return ::std::move(result.returnValue);
    }

    template<template<typename> class P = ::std::promise>
    auto getLaserAndBStateDataAsync(const ::Ice::Context& context = Ice::noExplicitContext)
        -> decltype(::std::declval<P<::RoboCompLaser::Laser::GetLaserAndBStateDataResult>>().get_future())
    {
        return _makePromiseOutgoing<::RoboCompLaser::Laser::GetLaserAndBStateDataResult, P>(false, this, &RoboCompLaser::LaserPrx::_iceI_getLaserAndBStateData, context);
    }

    ::std::function<void()>
    getLaserAndBStateDataAsync(::std::function<void(::RoboCompLaser::TLaserData, ::RoboCompGenericBase::TBaseState)> response,
                               ::std::function<void(::std::exception_ptr)> ex = nullptr,
                               ::std::function<void(bool)> sent = nullptr,
                               const ::Ice::Context& context = Ice::noExplicitContext)
    {
        auto responseCb = [response](::RoboCompLaser::Laser::GetLaserAndBStateDataResult&& result)
        {
            response(::std::move(result.returnValue), ::std::move(result.bState));
        };
        return _makeLamdaOutgoing<::RoboCompLaser::Laser::GetLaserAndBStateDataResult>(responseCb, ex, sent, this, &RoboCompLaser::LaserPrx::_iceI_getLaserAndBStateData, context);
    }

    void _iceI_getLaserAndBStateData(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<::RoboCompLaser::Laser::GetLaserAndBStateDataResult>>&, const ::Ice::Context&);

    ::RoboCompLaser::LaserConfData getLaserConfData(const ::Ice::Context& context = Ice::noExplicitContext)
    {
        return _makePromiseOutgoing<::RoboCompLaser::LaserConfData>(true, this, &RoboCompLaser::LaserPrx::_iceI_getLaserConfData, context).get();
    }

    template<template<typename> class P = ::std::promise>
    auto getLaserConfDataAsync(const ::Ice::Context& context = Ice::noExplicitContext)
        -> decltype(::std::declval<P<::RoboCompLaser::LaserConfData>>().get_future())
    {
        return _makePromiseOutgoing<::RoboCompLaser::LaserConfData, P>(false, this, &RoboCompLaser::LaserPrx::_iceI_getLaserConfData, context);
    }

    ::std::function<void()>
    getLaserConfDataAsync(::std::function<void(::RoboCompLaser::LaserConfData)> response,
                          ::std::function<void(::std::exception_ptr)> ex = nullptr,
                          ::std::function<void(bool)> sent = nullptr,
                          const ::Ice::Context& context = Ice::noExplicitContext)
    {
        return _makeLamdaOutgoing<::RoboCompLaser::LaserConfData>(response, ex, sent, this, &RoboCompLaser::LaserPrx::_iceI_getLaserConfData, context);
    }

    void _iceI_getLaserConfData(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<::RoboCompLaser::LaserConfData>>&, const ::Ice::Context&);

    ::RoboCompLaser::TLaserData getLaserData(const ::Ice::Context& context = Ice::noExplicitContext)
    {
        return _makePromiseOutgoing<::RoboCompLaser::TLaserData>(true, this, &RoboCompLaser::LaserPrx::_iceI_getLaserData, context).get();
    }

    template<template<typename> class P = ::std::promise>
    auto getLaserDataAsync(const ::Ice::Context& context = Ice::noExplicitContext)
        -> decltype(::std::declval<P<::RoboCompLaser::TLaserData>>().get_future())
    {
        return _makePromiseOutgoing<::RoboCompLaser::TLaserData, P>(false, this, &RoboCompLaser::LaserPrx::_iceI_getLaserData, context);
    }

    ::std::function<void()>
    getLaserDataAsync(::std::function<void(::RoboCompLaser::TLaserData)> response,
                      ::std::function<void(::std::exception_ptr)> ex = nullptr,
                      ::std::function<void(bool)> sent = nullptr,
                      const ::Ice::Context& context = Ice::noExplicitContext)
    {
        return _makeLamdaOutgoing<::RoboCompLaser::TLaserData>(response, ex, sent, this, &RoboCompLaser::LaserPrx::_iceI_getLaserData, context);
    }

    void _iceI_getLaserData(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<::RoboCompLaser::TLaserData>>&, const ::Ice::Context&);

    static const ::std::string& ice_staticId();

protected:

    LaserPrx() = default;
    friend ::std::shared_ptr<LaserPrx> IceInternal::createProxy<LaserPrx>();

    virtual ::std::shared_ptr<::Ice::ObjectPrx> _newInstance() const override;
};

}

namespace Ice
{

template<>
struct StreamableTraits<::RoboCompLaser::LaserConfData>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 46;
    static const bool fixedLength = false;
};

template<typename S>
struct StreamReader<::RoboCompLaser::LaserConfData, S>
{
    static void read(S* istr, ::RoboCompLaser::LaserConfData& v)
    {
        istr->readAll(v.driver, v.device, v.staticConf, v.maxMeasures, v.maxDegrees, v.maxRange, v.minRange, v.iniRange, v.endRange, v.cluster, v.sampleRate, v.angleRes, v.angleIni);
    }
};

template<>
struct StreamableTraits<::RoboCompLaser::TData>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 8;
    static const bool fixedLength = true;
};

template<typename S>
struct StreamReader<::RoboCompLaser::TData, S>
{
    static void read(S* istr, ::RoboCompLaser::TData& v)
    {
        istr->readAll(v.angle, v.dist);
    }
};

}

namespace RoboCompLaser
{

using LaserPtr = ::std::shared_ptr<Laser>;
using LaserPrxPtr = ::std::shared_ptr<LaserPrx>;

}

#else // C++98 mapping

namespace IceProxy
{

namespace RoboCompLaser
{

class Laser;
void _readProxy(::Ice::InputStream*, ::IceInternal::ProxyHandle< ::IceProxy::RoboCompLaser::Laser>&);
::IceProxy::Ice::Object* upCast(::IceProxy::RoboCompLaser::Laser*);

}

}

namespace RoboCompLaser
{

class Laser;
::Ice::Object* upCast(::RoboCompLaser::Laser*);
typedef ::IceInternal::Handle< ::RoboCompLaser::Laser> LaserPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompLaser::Laser> LaserPrx;
typedef LaserPrx LaserPrxPtr;
void _icePatchObjectPtr(LaserPtr&, const ::Ice::ObjectPtr&);

}

namespace RoboCompLaser
{

typedef ::std::vector< ::Ice::Int> shortVector;

struct LaserConfData
{
    ::std::string driver;
    ::std::string device;
    ::Ice::Int staticConf;
    ::Ice::Int maxMeasures;
    ::Ice::Int maxDegrees;
    ::Ice::Int maxRange;
    ::Ice::Int minRange;
    ::Ice::Int iniRange;
    ::Ice::Int endRange;
    ::Ice::Int cluster;
    ::Ice::Int sampleRate;
    ::Ice::Float angleRes;
    ::Ice::Float angleIni;
};

struct TData
{
    ::Ice::Float angle;
    ::Ice::Float dist;
};

typedef ::std::vector< ::RoboCompLaser::TData> TLaserData;

}

namespace RoboCompLaser
{

class Callback_Laser_getLaserAndBStateData_Base : public virtual ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_Laser_getLaserAndBStateData_Base> Callback_Laser_getLaserAndBStateDataPtr;

class Callback_Laser_getLaserConfData_Base : public virtual ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_Laser_getLaserConfData_Base> Callback_Laser_getLaserConfDataPtr;

class Callback_Laser_getLaserData_Base : public virtual ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_Laser_getLaserData_Base> Callback_Laser_getLaserDataPtr;

}

namespace IceProxy
{

namespace RoboCompLaser
{

class Laser : public virtual ::Ice::Proxy<Laser, ::IceProxy::Ice::Object>
{
public:

    ::RoboCompLaser::TLaserData getLaserAndBStateData(::RoboCompGenericBase::TBaseState& iceP_bState, const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return end_getLaserAndBStateData(iceP_bState, _iceI_begin_getLaserAndBStateData(context, ::IceInternal::dummyCallback, 0, true));
    }

    ::Ice::AsyncResultPtr begin_getLaserAndBStateData(const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _iceI_begin_getLaserAndBStateData(context, ::IceInternal::dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getLaserAndBStateData(const ::Ice::CallbackPtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getLaserAndBStateData(::Ice::noExplicitContext, del, cookie);
    }

    ::Ice::AsyncResultPtr begin_getLaserAndBStateData(const ::Ice::Context& context, const ::Ice::CallbackPtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getLaserAndBStateData(context, del, cookie);
    }

    ::Ice::AsyncResultPtr begin_getLaserAndBStateData(const ::RoboCompLaser::Callback_Laser_getLaserAndBStateDataPtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getLaserAndBStateData(::Ice::noExplicitContext, del, cookie);
    }

    ::Ice::AsyncResultPtr begin_getLaserAndBStateData(const ::Ice::Context& context, const ::RoboCompLaser::Callback_Laser_getLaserAndBStateDataPtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getLaserAndBStateData(context, del, cookie);
    }

    ::RoboCompLaser::TLaserData end_getLaserAndBStateData(::RoboCompGenericBase::TBaseState& iceP_bState, const ::Ice::AsyncResultPtr&);

private:

    ::Ice::AsyncResultPtr _iceI_begin_getLaserAndBStateData(const ::Ice::Context&, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& cookie = 0, bool sync = false);

public:

    ::RoboCompLaser::LaserConfData getLaserConfData(const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return end_getLaserConfData(_iceI_begin_getLaserConfData(context, ::IceInternal::dummyCallback, 0, true));
    }

    ::Ice::AsyncResultPtr begin_getLaserConfData(const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _iceI_begin_getLaserConfData(context, ::IceInternal::dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getLaserConfData(const ::Ice::CallbackPtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getLaserConfData(::Ice::noExplicitContext, del, cookie);
    }

    ::Ice::AsyncResultPtr begin_getLaserConfData(const ::Ice::Context& context, const ::Ice::CallbackPtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getLaserConfData(context, del, cookie);
    }

    ::Ice::AsyncResultPtr begin_getLaserConfData(const ::RoboCompLaser::Callback_Laser_getLaserConfDataPtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getLaserConfData(::Ice::noExplicitContext, del, cookie);
    }

    ::Ice::AsyncResultPtr begin_getLaserConfData(const ::Ice::Context& context, const ::RoboCompLaser::Callback_Laser_getLaserConfDataPtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getLaserConfData(context, del, cookie);
    }

    ::RoboCompLaser::LaserConfData end_getLaserConfData(const ::Ice::AsyncResultPtr&);

private:

    ::Ice::AsyncResultPtr _iceI_begin_getLaserConfData(const ::Ice::Context&, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& cookie = 0, bool sync = false);

public:

    ::RoboCompLaser::TLaserData getLaserData(const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return end_getLaserData(_iceI_begin_getLaserData(context, ::IceInternal::dummyCallback, 0, true));
    }

    ::Ice::AsyncResultPtr begin_getLaserData(const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _iceI_begin_getLaserData(context, ::IceInternal::dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getLaserData(const ::Ice::CallbackPtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getLaserData(::Ice::noExplicitContext, del, cookie);
    }

    ::Ice::AsyncResultPtr begin_getLaserData(const ::Ice::Context& context, const ::Ice::CallbackPtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getLaserData(context, del, cookie);
    }

    ::Ice::AsyncResultPtr begin_getLaserData(const ::RoboCompLaser::Callback_Laser_getLaserDataPtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getLaserData(::Ice::noExplicitContext, del, cookie);
    }

    ::Ice::AsyncResultPtr begin_getLaserData(const ::Ice::Context& context, const ::RoboCompLaser::Callback_Laser_getLaserDataPtr& del, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_getLaserData(context, del, cookie);
    }

    ::RoboCompLaser::TLaserData end_getLaserData(const ::Ice::AsyncResultPtr&);

private:

    ::Ice::AsyncResultPtr _iceI_begin_getLaserData(const ::Ice::Context&, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& cookie = 0, bool sync = false);

public:

    static const ::std::string& ice_staticId();

protected:

    virtual ::IceProxy::Ice::Object* _newInstance() const;
};

}

}

namespace RoboCompLaser
{

class Laser : public virtual ::Ice::Object
{
public:

    typedef LaserPrx ProxyType;
    typedef LaserPtr PointerType;

    virtual ~Laser();

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::emptyCurrent) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::emptyCurrent) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::emptyCurrent) const;

    static const ::std::string& ice_staticId();

    virtual ::RoboCompLaser::TLaserData getLaserAndBStateData(::RoboCompGenericBase::TBaseState&, const ::Ice::Current& = ::Ice::emptyCurrent) = 0;
    bool _iceD_getLaserAndBStateData(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::RoboCompLaser::LaserConfData getLaserConfData(const ::Ice::Current& = ::Ice::emptyCurrent) = 0;
    bool _iceD_getLaserConfData(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::RoboCompLaser::TLaserData getLaserData(const ::Ice::Current& = ::Ice::emptyCurrent) = 0;
    bool _iceD_getLaserData(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual bool _iceDispatch(::IceInternal::Incoming&, const ::Ice::Current&);

protected:

    virtual void _iceWriteImpl(::Ice::OutputStream*) const;
    virtual void _iceReadImpl(::Ice::InputStream*);
};

inline bool operator==(const Laser& lhs, const Laser& rhs)
{
    return static_cast<const ::Ice::Object&>(lhs) == static_cast<const ::Ice::Object&>(rhs);
}

inline bool operator<(const Laser& lhs, const Laser& rhs)
{
    return static_cast<const ::Ice::Object&>(lhs) < static_cast<const ::Ice::Object&>(rhs);
}

}

namespace Ice
{

template<>
struct StreamableTraits< ::RoboCompLaser::LaserConfData>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 46;
    static const bool fixedLength = false;
};

template<typename S>
struct StreamWriter< ::RoboCompLaser::LaserConfData, S>
{
    static void write(S* ostr, const ::RoboCompLaser::LaserConfData& v)
    {
        ostr->write(v.driver);
        ostr->write(v.device);
        ostr->write(v.staticConf);
        ostr->write(v.maxMeasures);
        ostr->write(v.maxDegrees);
        ostr->write(v.maxRange);
        ostr->write(v.minRange);
        ostr->write(v.iniRange);
        ostr->write(v.endRange);
        ostr->write(v.cluster);
        ostr->write(v.sampleRate);
        ostr->write(v.angleRes);
        ostr->write(v.angleIni);
    }
};

template<typename S>
struct StreamReader< ::RoboCompLaser::LaserConfData, S>
{
    static void read(S* istr, ::RoboCompLaser::LaserConfData& v)
    {
        istr->read(v.driver);
        istr->read(v.device);
        istr->read(v.staticConf);
        istr->read(v.maxMeasures);
        istr->read(v.maxDegrees);
        istr->read(v.maxRange);
        istr->read(v.minRange);
        istr->read(v.iniRange);
        istr->read(v.endRange);
        istr->read(v.cluster);
        istr->read(v.sampleRate);
        istr->read(v.angleRes);
        istr->read(v.angleIni);
    }
};

template<>
struct StreamableTraits< ::RoboCompLaser::TData>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 8;
    static const bool fixedLength = true;
};

template<typename S>
struct StreamWriter< ::RoboCompLaser::TData, S>
{
    static void write(S* ostr, const ::RoboCompLaser::TData& v)
    {
        ostr->write(v.angle);
        ostr->write(v.dist);
    }
};

template<typename S>
struct StreamReader< ::RoboCompLaser::TData, S>
{
    static void read(S* istr, ::RoboCompLaser::TData& v)
    {
        istr->read(v.angle);
        istr->read(v.dist);
    }
};

}

namespace RoboCompLaser
{

template<class T>
class CallbackNC_Laser_getLaserAndBStateData : public Callback_Laser_getLaserAndBStateData_Base, public ::IceInternal::TwowayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)(const ::RoboCompLaser::TLaserData&, const ::RoboCompGenericBase::TBaseState&);

    CallbackNC_Laser_getLaserAndBStateData(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallbackNC<T>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    virtual void completed(const ::Ice::AsyncResultPtr& result) const
    {
        ::RoboCompLaser::LaserPrx proxy = ::RoboCompLaser::LaserPrx::uncheckedCast(result->getProxy());
        ::RoboCompGenericBase::TBaseState iceP_bState;
        ::RoboCompLaser::TLaserData ret;
        try
        {
            ret = proxy->end_getLaserAndBStateData(iceP_bState, result);
        }
        catch(const ::Ice::Exception& ex)
        {
            ::IceInternal::CallbackNC<T>::exception(result, ex);
            return;
        }
        if(_response)
        {
            (::IceInternal::CallbackNC<T>::_callback.get()->*_response)(ret, iceP_bState);
        }
    }

private:

    Response _response;
};

template<class T> Callback_Laser_getLaserAndBStateDataPtr
newCallback_Laser_getLaserAndBStateData(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompLaser::TLaserData&, const ::RoboCompGenericBase::TBaseState&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Laser_getLaserAndBStateData<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_Laser_getLaserAndBStateDataPtr
newCallback_Laser_getLaserAndBStateData(T* instance, void (T::*cb)(const ::RoboCompLaser::TLaserData&, const ::RoboCompGenericBase::TBaseState&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Laser_getLaserAndBStateData<T>(instance, cb, excb, sentcb);
}

template<class T, typename CT>
class Callback_Laser_getLaserAndBStateData : public Callback_Laser_getLaserAndBStateData_Base, public ::IceInternal::TwowayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const ::RoboCompLaser::TLaserData&, const ::RoboCompGenericBase::TBaseState&, const CT&);

    Callback_Laser_getLaserAndBStateData(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallback<T, CT>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    virtual void completed(const ::Ice::AsyncResultPtr& result) const
    {
        ::RoboCompLaser::LaserPrx proxy = ::RoboCompLaser::LaserPrx::uncheckedCast(result->getProxy());
        ::RoboCompGenericBase::TBaseState iceP_bState;
        ::RoboCompLaser::TLaserData ret;
        try
        {
            ret = proxy->end_getLaserAndBStateData(iceP_bState, result);
        }
        catch(const ::Ice::Exception& ex)
        {
            ::IceInternal::Callback<T, CT>::exception(result, ex);
            return;
        }
        if(_response)
        {
            (::IceInternal::Callback<T, CT>::_callback.get()->*_response)(ret, iceP_bState, CT::dynamicCast(result->getCookie()));
        }
    }

private:

    Response _response;
};

template<class T, typename CT> Callback_Laser_getLaserAndBStateDataPtr
newCallback_Laser_getLaserAndBStateData(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompLaser::TLaserData&, const ::RoboCompGenericBase::TBaseState&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Laser_getLaserAndBStateData<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_Laser_getLaserAndBStateDataPtr
newCallback_Laser_getLaserAndBStateData(T* instance, void (T::*cb)(const ::RoboCompLaser::TLaserData&, const ::RoboCompGenericBase::TBaseState&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Laser_getLaserAndBStateData<T, CT>(instance, cb, excb, sentcb);
}

template<class T>
class CallbackNC_Laser_getLaserConfData : public Callback_Laser_getLaserConfData_Base, public ::IceInternal::TwowayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)(const ::RoboCompLaser::LaserConfData&);

    CallbackNC_Laser_getLaserConfData(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallbackNC<T>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    virtual void completed(const ::Ice::AsyncResultPtr& result) const
    {
        ::RoboCompLaser::LaserPrx proxy = ::RoboCompLaser::LaserPrx::uncheckedCast(result->getProxy());
        ::RoboCompLaser::LaserConfData ret;
        try
        {
            ret = proxy->end_getLaserConfData(result);
        }
        catch(const ::Ice::Exception& ex)
        {
            ::IceInternal::CallbackNC<T>::exception(result, ex);
            return;
        }
        if(_response)
        {
            (::IceInternal::CallbackNC<T>::_callback.get()->*_response)(ret);
        }
    }

private:

    Response _response;
};

template<class T> Callback_Laser_getLaserConfDataPtr
newCallback_Laser_getLaserConfData(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompLaser::LaserConfData&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Laser_getLaserConfData<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_Laser_getLaserConfDataPtr
newCallback_Laser_getLaserConfData(T* instance, void (T::*cb)(const ::RoboCompLaser::LaserConfData&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Laser_getLaserConfData<T>(instance, cb, excb, sentcb);
}

template<class T, typename CT>
class Callback_Laser_getLaserConfData : public Callback_Laser_getLaserConfData_Base, public ::IceInternal::TwowayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const ::RoboCompLaser::LaserConfData&, const CT&);

    Callback_Laser_getLaserConfData(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallback<T, CT>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    virtual void completed(const ::Ice::AsyncResultPtr& result) const
    {
        ::RoboCompLaser::LaserPrx proxy = ::RoboCompLaser::LaserPrx::uncheckedCast(result->getProxy());
        ::RoboCompLaser::LaserConfData ret;
        try
        {
            ret = proxy->end_getLaserConfData(result);
        }
        catch(const ::Ice::Exception& ex)
        {
            ::IceInternal::Callback<T, CT>::exception(result, ex);
            return;
        }
        if(_response)
        {
            (::IceInternal::Callback<T, CT>::_callback.get()->*_response)(ret, CT::dynamicCast(result->getCookie()));
        }
    }

private:

    Response _response;
};

template<class T, typename CT> Callback_Laser_getLaserConfDataPtr
newCallback_Laser_getLaserConfData(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompLaser::LaserConfData&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Laser_getLaserConfData<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_Laser_getLaserConfDataPtr
newCallback_Laser_getLaserConfData(T* instance, void (T::*cb)(const ::RoboCompLaser::LaserConfData&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Laser_getLaserConfData<T, CT>(instance, cb, excb, sentcb);
}

template<class T>
class CallbackNC_Laser_getLaserData : public Callback_Laser_getLaserData_Base, public ::IceInternal::TwowayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)(const ::RoboCompLaser::TLaserData&);

    CallbackNC_Laser_getLaserData(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallbackNC<T>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    virtual void completed(const ::Ice::AsyncResultPtr& result) const
    {
        ::RoboCompLaser::LaserPrx proxy = ::RoboCompLaser::LaserPrx::uncheckedCast(result->getProxy());
        ::RoboCompLaser::TLaserData ret;
        try
        {
            ret = proxy->end_getLaserData(result);
        }
        catch(const ::Ice::Exception& ex)
        {
            ::IceInternal::CallbackNC<T>::exception(result, ex);
            return;
        }
        if(_response)
        {
            (::IceInternal::CallbackNC<T>::_callback.get()->*_response)(ret);
        }
    }

private:

    Response _response;
};

template<class T> Callback_Laser_getLaserDataPtr
newCallback_Laser_getLaserData(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompLaser::TLaserData&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Laser_getLaserData<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_Laser_getLaserDataPtr
newCallback_Laser_getLaserData(T* instance, void (T::*cb)(const ::RoboCompLaser::TLaserData&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Laser_getLaserData<T>(instance, cb, excb, sentcb);
}

template<class T, typename CT>
class Callback_Laser_getLaserData : public Callback_Laser_getLaserData_Base, public ::IceInternal::TwowayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const ::RoboCompLaser::TLaserData&, const CT&);

    Callback_Laser_getLaserData(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallback<T, CT>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    virtual void completed(const ::Ice::AsyncResultPtr& result) const
    {
        ::RoboCompLaser::LaserPrx proxy = ::RoboCompLaser::LaserPrx::uncheckedCast(result->getProxy());
        ::RoboCompLaser::TLaserData ret;
        try
        {
            ret = proxy->end_getLaserData(result);
        }
        catch(const ::Ice::Exception& ex)
        {
            ::IceInternal::Callback<T, CT>::exception(result, ex);
            return;
        }
        if(_response)
        {
            (::IceInternal::Callback<T, CT>::_callback.get()->*_response)(ret, CT::dynamicCast(result->getCookie()));
        }
    }

private:

    Response _response;
};

template<class T, typename CT> Callback_Laser_getLaserDataPtr
newCallback_Laser_getLaserData(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompLaser::TLaserData&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Laser_getLaserData<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_Laser_getLaserDataPtr
newCallback_Laser_getLaserData(T* instance, void (T::*cb)(const ::RoboCompLaser::TLaserData&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Laser_getLaserData<T, CT>(instance, cb, excb, sentcb);
}

}

#endif

#include <IceUtil/PopDisableWarnings.h>
#endif
