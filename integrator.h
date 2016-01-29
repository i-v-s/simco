#ifndef INTEGRATOR_H
#define INTEGRATOR_H
#include <cstddef>
#include <assert.h>
#include <vector>
#include <memory>
namespace itg {

typedef double real;

class Ref
{
    size_t _size;
    real * _ptr;
public:
    inline size_t size() {return _size;}
    inline void setPtr(real * ptr) { _ptr = ptr;}
    inline real &operator[](size_t idx)
    {
        assert(_ptr);
        assert(idx < _size);
        return _ptr[idx];
    }
    inline const real &operator[](size_t idx) const
    {
        assert(_ptr);
        assert(idx < _size);
        return _ptr[idx];
    }
    Ref(size_t size): _size(size), _ptr(0) {};
};

class VectorRef;

typedef std::unique_ptr<VectorRef> VectorRefPtr;

class VectorRef
{
public:
    std::vector<VectorRefPtr> _subVectors;
    std::vector<Ref *> _vars;
    real * initialize(real * ptr)
    {
        for(auto v : _vars)
        {
            v->setPtr(ptr);
            ptr += v->size();
        }
        return ptr;
    }

    VectorRef(const std::vector<Ref *> & vars): _vars(vars) {};
    VectorRef() {};
    virtual ~VectorRef() {};
};

//template<class T>

class Model
{
private:

public:
    virtual void f(VectorRef * d, const VectorRef * x) = 0;
    virtual size_t vectorSize() = 0;
    virtual VectorRef * createVector() = 0;
    //Model()
};

template<class T> class ModelT : public Model
{
public:
    virtual void f(VectorRef * d, const VectorRef * x)
    {
        f(* static_cast<T *>(d), * static_cast<const T *>(x));
    }
    virtual void f(T &d, const T &x) = 0;
    virtual void getInitialState(T & x) = 0;
    VectorRef * createVector() { return new T();}

    size_t vectorSize()
    {
        T ref;
        int size = 0;
        for(auto v : ref._vars)
            size += v->size();
        return size;
    }
};


class Vector: public VectorRef, public std::vector<real>
{
public:
    //Vector operator * (itg::real m)
    const Vector & operator += (const std::vector<real> & a)
    {
        assert(size() == a.size());
        auto p = begin();
        for(auto ap = a.begin(); p != end(); ++p, ++ap)
            *p += *ap;
        return *this;
    }

};

class Integrator
{
protected:
    real _time;
    void f(VectorRef & d, const VectorRef & x)
    {
        auto dv = d._subVectors.begin();
        auto xv = x._subVectors.begin();
        for(auto m : _models)
        {
            m->f((dv++)->get(), (xv++)->get());
        }
    }
    void initializeVector(Vector & v)
    {
        size_t size = 0;
        for(auto m : _models)
        {
            size += m->vectorSize();
            v._subVectors.push_back(VectorRefPtr(m->createVector()));
        }
        v.resize(size);
        /*int t = 0;
        for(auto x = v.begin(); x != v.end(); x++) *x = t++;*/
        real * ptr = v.data();
        for(auto sv = v._subVectors.begin(); sv != v._subVectors.end(); sv++)
            ptr = (*sv)->initialize(ptr);
    }
public:
    std::vector<Model *> _models;
    inline void setTime(real time) { _time = time;}
    virtual void stepTo(real time) = 0;
    Integrator();

    virtual void initialize() = 0;
};

class IntegratorEuler: public Integrator
{
private:
    Vector _x;
    Vector _dx;
public:
    void initialize()
    {
        initializeVector(_x);
        initializeVector(_dx);
    }
    void stepTo(real time)
    {
        f(_dx, _x);
        //_x += _dx;// * (time - _time);
        real dt = time - _time;
        for(auto x = _x.begin(), dx = _dx.begin(); x != _x.end(); x++, dx++)
            *x += *dx * dt;
        _time = time;
    }

};

}
#endif // INTEGRATOR_H
