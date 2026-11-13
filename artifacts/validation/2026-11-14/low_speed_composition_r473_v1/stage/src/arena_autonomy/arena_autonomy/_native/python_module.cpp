// NumPy C API 의존성 없이 연속 buffer만 사용한다. 명시 선택 자식 계산용.
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <cstdint>
#include <cstring>

extern "C" int arena_depth_evaluate(int64_t,int64_t,int64_t,int64_t,const double*,const int64_t*,
    const double*,const double*,const int64_t*,const uint8_t*,const double*,const double*,uint8_t*);
extern "C" int arena_polygon_distance(int64_t,int64_t,int,const double*,const double*,double*);
extern "C" int arena_masked_evaluate(int64_t,int64_t,int64_t,const double*,const int64_t*,
    const double*,const double*,const int64_t*,const uint8_t*,const double*,const double*,uint8_t*);
extern "C" int arena_support_classes(int64_t,int64_t,int64_t,int64_t,const double*,const int64_t*,
    const double*,const double*,const int64_t*,const uint8_t*,const double*,const double*,uint8_t*);

extern "C" int arena_current_mask_occupied(int64_t,int64_t,const double*,const double*,const double*,uint8_t*);

struct Buffers {
    Py_buffer views[9]{};
    ~Buffers(){for(auto &v:views)if(v.obj)PyBuffer_Release(&v);}
    bool get(PyObject* args,int offset,int count,const int64_t* sizes,const char* types) {
        for(int i=0;i<count;++i) {
            const int flags=PyBUF_FORMAT|PyBUF_C_CONTIGUOUS|(i==count-1 ? PyBUF_WRITABLE : 0);
            auto &v=views[i];
            if(PyObject_GetBuffer(PyTuple_GET_ITEM(args,offset+i),&v,flags)<0)return false;
            const char* format=v.format ? v.format : "";
            bool type=types[i]=='d' ? std::strcmp(format,"d")==0 :
                types[i]=='B' ? std::strcmp(format,"B")==0 :
                (std::strcmp(format,"l")==0 || std::strcmp(format,"q")==0);
            int unit=types[i]=='B' ? 1 : 8;
            if(!type || v.itemsize!=unit || v.len!=sizes[i]*unit) {
                PyErr_SetString(PyExc_ValueError,"numeric kernel buffer shape/type mismatch");return false;
            }
        }
        return true;
    }
};

static PyObject* depth(PyObject*,PyObject* args) {
    if(PyTuple_Size(args)!=13){PyErr_SetString(PyExc_TypeError,"depth requires four sizes and nine buffers");return nullptr;}
    int64_t n[4];
    for(int i=0;i<4;++i){n[i]=PyLong_AsLongLong(PyTuple_GET_ITEM(args,i));if(PyErr_Occurred())return nullptr;}
    if(n[0]<0 || n[0]>1000000 || n[1]<1 || n[1]>16384 || n[2]<1 || n[2]>16384 || n[3]<-1 || n[3]>16384) {
        PyErr_SetString(PyExc_ValueError,"bounded numeric kernel sizes required");return nullptr;
    }
    int64_t sizes[]={2*n[0],n[0]*n[1],2*n[2],2*n[2],n[2],n[2],n[2],n[2],n[0]};
    Buffers b;if(!b.get(args,4,9,sizes,"dqddqBddB"))return nullptr;
    int result;
    Py_BEGIN_ALLOW_THREADS
    result=arena_depth_evaluate(n[0],n[1],n[2],n[3],(double*)b.views[0].buf,(int64_t*)b.views[1].buf,
        (double*)b.views[2].buf,(double*)b.views[3].buf,(int64_t*)b.views[4].buf,(uint8_t*)b.views[5].buf,
        (double*)b.views[6].buf,(double*)b.views[7].buf,(uint8_t*)b.views[8].buf);
    Py_END_ALLOW_THREADS
    if(result){PyErr_SetString(PyExc_ValueError,"numeric depth index outside rays");return nullptr;}
    Py_RETURN_NONE;
}

static PyObject* masked(PyObject*,PyObject* args) {
    if(PyTuple_Size(args)!=12){PyErr_SetString(PyExc_TypeError,"masked requires three sizes and nine buffers");return nullptr;}
    int64_t n[3];
    for(int i=0;i<3;++i){n[i]=PyLong_AsLongLong(PyTuple_GET_ITEM(args,i));if(PyErr_Occurred())return nullptr;}
    if(n[0]<0 || n[0]>1000000 || n[1]<1 || n[1]>16384 || n[2]<1 || n[2]>16384) {
        PyErr_SetString(PyExc_ValueError,"bounded masked kernel sizes required");return nullptr;
    }
    int64_t sizes[]={2*n[0],n[0]*n[1],2*n[2],2*n[2],n[2],n[2],n[2],n[2],n[0]};
    Buffers b;if(!b.get(args,3,9,sizes,"dqddqBddB"))return nullptr;
    int result;
    Py_BEGIN_ALLOW_THREADS
    result=arena_masked_evaluate(n[0],n[1],n[2],(double*)b.views[0].buf,(int64_t*)b.views[1].buf,
        (double*)b.views[2].buf,(double*)b.views[3].buf,(int64_t*)b.views[4].buf,(uint8_t*)b.views[5].buf,
        (double*)b.views[6].buf,(double*)b.views[7].buf,(uint8_t*)b.views[8].buf);
    Py_END_ALLOW_THREADS
    if(result){PyErr_SetString(PyExc_ValueError,"numeric masked index outside rays");return nullptr;}
    Py_RETURN_NONE;
}

static PyObject* classes(PyObject*,PyObject* args) {
    if(PyTuple_Size(args)!=13){PyErr_SetString(PyExc_TypeError,"classes requires four sizes and nine buffers");return nullptr;}
    int64_t n[4];
    for(int i=0;i<4;++i){n[i]=PyLong_AsLongLong(PyTuple_GET_ITEM(args,i));if(PyErr_Occurred())return nullptr;}
    if(n[0]<0 || n[0]>1000000 || n[1]<1 || n[1]>16384 || n[2]<1 || n[2]>16384 || n[3]<-1 || n[3]>16384){
        PyErr_SetString(PyExc_ValueError,"bounded classification sizes required");return nullptr;
    }
    int64_t sizes[]={2*n[0],n[0]*n[1],2*n[2],2*n[2],n[2],n[2],n[2],n[2],n[0]};
    Buffers b;if(!b.get(args,4,9,sizes,"dqddqBddB"))return nullptr;
    int result;
    Py_BEGIN_ALLOW_THREADS
    result=arena_support_classes(n[0],n[1],n[2],n[3],(double*)b.views[0].buf,(int64_t*)b.views[1].buf,
        (double*)b.views[2].buf,(double*)b.views[3].buf,(int64_t*)b.views[4].buf,(uint8_t*)b.views[5].buf,
        (double*)b.views[6].buf,(double*)b.views[7].buf,(uint8_t*)b.views[8].buf);
    Py_END_ALLOW_THREADS
    if(result){PyErr_SetString(PyExc_ValueError,"numeric classification index outside rays");return nullptr;}
    Py_RETURN_NONE;
}

static PyObject* polygon(PyObject*,PyObject* args) {
    if(PyTuple_Size(args)!=6){PyErr_SetString(PyExc_TypeError,"polygon requires three sizes and three buffers");return nullptr;}
    int64_t n[3];
    for(int i=0;i<3;++i){n[i]=PyLong_AsLongLong(PyTuple_GET_ITEM(args,i));if(PyErr_Occurred())return nullptr;}
    if(n[0]<0 || n[0]>1000000 || n[1]<1 || n[1]>16384 || n[2]<0 || n[2]>1) {
        PyErr_SetString(PyExc_ValueError,"bounded polygon kernel sizes required");return nullptr;
    }
    int64_t sizes[]={2*n[0],2*n[1],n[0]};Buffers b;
    if(!b.get(args,3,3,sizes,"ddd"))return nullptr;
    int result;
    Py_BEGIN_ALLOW_THREADS
    result=arena_polygon_distance(n[0],n[1],n[2],(double*)b.views[0].buf,(double*)b.views[1].buf,(double*)b.views[2].buf);
    Py_END_ALLOW_THREADS
    if(result){PyErr_SetString(PyExc_ValueError,"numeric polygon input invalid");return nullptr;}
    Py_RETURN_NONE;
}

static PyObject* current_mask_occupied(PyObject*,PyObject* args) {
    if(PyTuple_Size(args)!=6){PyErr_SetString(PyExc_TypeError,"current_mask_occupied requires two sizes and four buffers");return nullptr;}
    int64_t n[2];
    for(int i=0;i<2;++i){n[i]=PyLong_AsLongLong(PyTuple_GET_ITEM(args,i));if(PyErr_Occurred())return nullptr;}
    if(n[0]<0 || n[0]>1000000 || n[1]<0 || n[1]>16384){
        PyErr_SetString(PyExc_ValueError,"bounded current mask occupancy sizes required");return nullptr;
    }
    int64_t sizes[]={2*n[0],2*n[1],n[1],1};Buffers b;
    if(!b.get(args,2,4,sizes,"dddB"))return nullptr;
    int result;
    Py_BEGIN_ALLOW_THREADS
    result=arena_current_mask_occupied(n[0],n[1],(double*)b.views[0].buf,
        (double*)b.views[1].buf,(double*)b.views[2].buf,(uint8_t*)b.views[3].buf);
    Py_END_ALLOW_THREADS
    if(result){PyErr_SetString(PyExc_ValueError,"numeric current mask occupancy input invalid");return nullptr;}
    Py_RETURN_NONE;
}

static PyMethodDef methods[]={{"depth",depth,METH_VARARGS,"Bounded angular support kernel"},
    {"masked",masked,METH_VARARGS,"Self-masked classification with unchanged NumPy semantics"},
    {"classes",classes,METH_VARARGS,"Shared free and masked angular search, independent rules"},
    {"current_mask_occupied",current_mask_occupied,METH_VARARGS,"Exact inclusive known-return padding predicate"},
    {"polygon",polygon,METH_VARARGS,"Point to polygon distance kernel"},{nullptr,nullptr,0,nullptr}};
static PyModuleDef module={PyModuleDef_HEAD_INIT,"_numeric_kernels",nullptr,-1,methods};
PyMODINIT_FUNC PyInit__numeric_kernels(){
    PyObject* result=PyModule_Create(&module);
    if(result)PyModule_AddIntConstant(result,"ABI_VERSION",1);
    return result;
}
