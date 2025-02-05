# DGFSDK

DGF (Dense Geometry Format) is a block-based geometry compression technology developed by AMD.  It is a hardware-friendly format which will be directly supported by future AMD GPU Architectures.  For more information, refer to the [technical paper](https://gpuopen.com/download/publications/DGF.pdf).

This repository contains our DGF encoding toolchain.  The directory structure is as follows:

* **DGFLib**:  A library containing the low-level encoder/decoder for DGF blocks
* **DGFBaker**: A library which implements a DGF content baking pipeline
* **DGFTester**: A command-line test harness
* **DGFSample**: A simple D3D12 viewer for DGF models, which demonstrates real-time decoding in HLSL shaders. 
   
# Building the SDK

To build the SDK:

```
git clone <repository URL>
cd <repository_root_on_disk>
mkdir build
cd build
cmake ..
// compile using your build system of choice
```

To integrate the SDK into a larger cmake project, point cmake at it and set the corresponding variables to indicate what you want to include.  For example, to build only the DGFBaker and DGFLib, one would do this:

```
set( DGF_BUILD_DGFLIB, 1 )
set( DGF_BUILD_DGFBAKER, 1 )
set( DGF_BUILD_DGFTESTER, 0 )
set( DGF_BUILD_SAMPLES, 0 )
add_subdirectory ( ${PATH_TO_DGFSDK} );
```

## Using DGFLib

The `DGFLib` is a single header/CPP pair which contains functions for manipulating DGF blocks.  

The following example illustrates how to use DGFLib to decode a DGF block:

```C
    // read the encoding parameters
    DGF::MetaData meta;
    DGF::DecodeMetaData( &meta, pBlock );

    // unpack the triangle strip
    DGF::TriControlValues controlValues[DGF::MAX_TRIS];
    uint8_t stripIndexBuffer[DGF::MAX_INDICES];
    DecodeTopology(controlValues, indexBuffer, block);
    
    // convert the triangle strip to a triangle list
    uint8_t triangleList[DGF::MAX_INDICES];
    DGF::ConvertTopologyToTriangleList( triangleList, controlValues, indexBuffer, meta.numTris );
 
    // decode per-triangle geometry IDs and opacity flags
    uint8_t opaqueFlags[DGF::MAX_TRIS];
    uint32_t geomID[DGF::MAX_TRIS];
    DGF::DecodeGeomIDs( geomID, opaqueFlags, pBlock );
    
    // unpack the vertex offsets
    DGF::OffsetVert offsetVerts[DGF::MAX_VERTS];
    DGF::DecodeOffsetVerts( meta.numVerts, offsetVerts, pBlock );
    
    // convert the vertex offsets to floating-point vertex positions
    DGF::FloatVert floatVerts[DGF::MAX_VERTS];
    DGF::ConvertOffsetsToFloat( meta.numVerts, floatVerts, offsetVerts, meta );
    
    // reconstruct the primitive IDs
    uint32_t primIDs[DGF::MAX_TRIS];
    for( size_t i=0; i<meta.numTris; i++ )
        primIDs[i] = meta.primIDBase + i;
```

### DGF_ASSERT

DGFLib has a custom assertion mechanism, which client code can override by injecting an "assert delegate".  The interface for this is shown below.

```C
namespace DGF
{
    typedef bool (*pfnAssertDelegate)( const char* File, int Line, const char* Condition );
    void SetAssertDelegate( pfnAssertDelegate filter );
}
```

DGF will ignore the assertion if the delegate returns false.  A default implementation is used if no assert delegate is provided.   DGFLib and DGFBaker are exception safe, so assert delegates may throw.  By default, DGF assertions are always enabled.  `DGF_NO_ASSERTS` may be added to the compile definitions to remove them completely.   The DGFBaker library is built on DGFLib and uses the same assertion mechanism.
		
## Using DGFBaker 

The `DGFBaker` library implements a reference pipeline for converting 3D geometry data to DGF.  

A simple example is shown below:

```C
#include <DGFBaker.h>

// Returns an array of 128B DGF blocks for the input geometry
std::vector<uint8_t> BakeDGF( const float* vertices, const uint32_t* indices, size_t numVerts, size_t numTris )
{
    DGFBaker::BakerConfig config = {};
    DGFBaker::Baker baker(config);
    
    DGFBaker::BakerMesh mesh(vertices, indices, numVerts, numTris);
    DGFBaker::BakerOutput output = baker.BakeDGF(mesh);
    
    return std::move(output.dgfBlocks);
}
```

The ``BakerMesh`` class is implemented using 'Reader' functions to enable flexibility in the input and output data formats.  The signatures of these reader functions are shown below:

```C
// Reads a set of vertices by index (3 floats per vertex)
typedef std::function<void(float*, const uint32_t* pVertexIndices, size_t numVertices)> VertexReader;

// Reads a range of triangle indices (3 indices per tri)
typedef std::function<void(uint32_t*, const uint32_t* pTriIndices, size_t numTris)> IndexReader;

// Reads a range of triangle attributes (1 per tri) for a set of indexed triangles
typedef std::function<void(DGFBaker::TriangleAttributes*, const uint32_t* triIndices, size_t numTris )> AttributeReader;
```

Custom reader functions can be written for specific use cases.  For example, a mesh with strided vertices and 16b indices could be constructed like this:

```C
std::vector<uint8_t> StridedMeshExample( 
       size_t vertexStride, 
       const uint8_t* vertexData, 
       const uint16_t* indices, 
       size_t numTris, size_t numVerts )
{
    auto _VertexReader = [vertexStride,vertexData]( float* output, const uint32_t* vertIndices, size_t numIndices )
    {
        for( size_t i=0; i<numIndices; i++ )
        {
	    uint32_t index = vertIndices[i];
	    memcpy( &output[3*index], vertexData + vertexStride*index, 3*sizeof(float) );
	}
    };
    auto _IndexReader = [indices]( uint32_t* output, const uint32_t* triIndices, size_t numTris )
    {
	for( size_t i=0; i<numTris; i++ )
        {
	    uint32_t triIndex = triIndices[i];
	    for( size_t j=0; j<3; j++ )
	        output[3*i+j] = indices[3*triIndex+j];
	}
    };
  
    DGFBaker::BakerMesh mesh( _VertexReader, _IndexReader, numVerts, numTris );
    DGFBaker::BakerOutput output = baker.BakeDGF(mesh);    
    return std::move(output.dgfBlocks);
}
```

### Baker Configuration

The DGFBaker uses a simple configuration structure shown below:

```C
  struct Config
  {
      size_t clusterMaxFaces = 128;
      size_t clusterMaxVerts = 256;
      size_t targetBitWidth = 16;     // target signed bit-width for the fixed-point vertex offsets.  Used to select quantization exponent
      size_t blockMaxTris  = 64;
      size_t blockMaxVerts = 64;
      uint8_t blockForcedOffsetWidth[3] = { 0,0,0 }; // if non-zero, bit-width of x,y,z offsets are forced to these values
                                                     //    if the vertex data needs more precision, data loss may occur.
                                                     // If the sum of the widths is not a multiple of 4, x,y,z are incremented until it is

      std::ostream* outputStream = nullptr;
      bool printPerfData = false;                     
      bool validateClusters = false;                 // Enable sanity-checks for debugging cluster formation
      bool generateVertexTable = false;              // Generate the vertex remapping table
      bool generateClusterTable = false;             // Generate the cluster table
      bool generateTriangleRemap = false;            // Generate the triangle remapping table
      bool enableUserData = false;                   // Reserve space for a 32-bit user-data field following the block headers
      bool encoderRoundTripValidation = false;       // Enable sanity-checks for debugging DGFLib encoder
      PackerMode packer = PackerMode::DEFAULT;

      bool enableExponentAdjust  = true;           // Allow block exponents to vary to improve compression for pre-quantized input
  };
    
```

The algorithms used in DGFBaker are described in the [HPG 2024 Paper](https://gpuopen.com/download/publications/DGF.pdf).  We refer the reader to this paper for additional context.  The most important fields are described below:

* *targetBitWidth*:  This corresponds to the `b` parameter from the HPG 2024 paper.  It is the target bit width for the per-vertex offsets, and controls the tradeoff between compression rate and vertex accuracy.
* *packer*:  Selects the block packing algorithm to use.  
* *enableUserData*: Controls whether to reserve space for a 32-bit user-data field in each block.  Setting this to false improves compression rate.
* *blockForcedOffsetWidth* : Force a particular width for the block offsets, to allow in-place update for animated blocks.

The supported packer modes are:

* `DGFBaker::PackerMode::HPG24`:  The algorithm described in the HPG 2024 paper
* `DGFBaker::PackerMode::SAH`:  An alternate algorithm which performs SAH splits until the resulting triangles fit in a single block.

The `SAH` packer mode is the recommended default.  The `HPG24` algorithm is provided as an alternative, and produces slightly better compression rates, but the resulting blocks have a larger surface area more vertex duplication, which makes them less suitable for ray tracing or fine-grained culling.

### Decoding

DGFBaker provides a convenient decoding pipeline built on DGFLib, as shown below:

```C
void DumpOBJFile( const float* vertices, const uint32_t* indices, size_t numVerts, size_t numIndices );

void DecodeAndDumpDGF( const DGFBaker::BakerOutput& output )
{
    DGFBaker::DecodedMesh decoded = DGFBaker::DecodeDGF(output);
    const float* vertices = decoded.GetVertexBuffer();  // 3 floats per vertex
    const uint32_t* indices = decoded.GetIndexBuffer(); // 3 indices per triangle
    size_t numVerts = decoded.GetVertexCount();
    size_t numTris = decoded.GetTriangleCount();
    DumpOBJFile( vertices, indices, numVerts, numTris ); 
}
```

### Validation

DGFBaker provides a helper class for verifying that the encoded blocks are a semantic match to the input, as shown below:

```C
bool IsItBroken( const DGFBaker::BakerMesh& input, const DGFBaker::BakerOutput& output )
{
    DGFBaker::Validator val(std::cout);
    if (!val.ValidateDGF(input, output))
    {
        return false;
    }
    return true;
}
```

### Triangle Sideband Data

The DGF baker will remove triangles for which two or more vertex indices are the same.  It will also re-order the remaining triangles in order to minimize the size of the compressed connectivity data.  If the application maintains sideband data such as index buffers, these must be post-processed to respect the new triangle order.  The baker can emit remapping information for this purpose.  The following example shows how the index buffer can be remapped.   This process is similar to the kinds of triangle reordering which content pipelines already perform for vertex cache optimization.

```C
namespace DGFBaker
{
    // Optional triangle remapping information
    //  This is not generated unless BakerConfig::generateTriangleRemap is set to true
    struct TriangleRemapInfo
    {
        uint32_t InputPrimIndex;    // For each output triangle, which input triangle was it?
        uint8_t IndexRotation[3];   // For each vertex of this triangle, which input vertex was it (0,1, or 2)
    };
}
    
size_t PostProcessIndexBuffer( 
    uint32_t* outputIndexBuffer,
    const uint32_t* inputIndexBuffer, 
    const DGFBaker::BakerOutput& output )
{
    const std::vector<DGFBaker::TriangleRemapInfo>& remap = output.triangleRemap;
    size_t numOutputTris = remap.size();
    for( size_t i=0; i<numOutputTris; i++ )
    {
        for( size_t j=0; j<3; j++ )
	{
	    uint32_t indexPos = 3*remap[i].InputPrimIndex + remap[i].IndexRotation[j];
	    outputIndexBuffer[3*i+j] = inputIndexBuffer[indexPos];
	}
    }
    return numOutputTris;
}
```

### Vertex Sideband Data ###

In order to render DGF data, it's necessary to retrieve per-vertex attributes at render time.  The simplest way to do this is to use a sideband index buffer which is indexed by the stored primitive IDs.  This imposes an overhead of up to 12 Bytes per triangle, which is several times larger than the DGF data itself.  This overhead can be reduced by duplicating vertex attributes across blocks, and re-using the existing connectivity data in the block to access the vertex attributes.

To facilitate this, the baker can emit a table containing the index of the input vertex for each vertex in each output block.  

```C
template< class VertexData_T > // VertexData_T is an application-defined attribute structure
void PostProcessVertexAttributeBuffer(  
    std::vector<VertexData_T>& outputAttributes,
    const std::vector<VertexData_T>& inputAttributes,
    DGFBaker::BakerOutput& bakerOutput )
{
    // NOTE:  The vertex table is not generated unless 'generateVertexTable' is set in the baker config
    const std::vector<uint32_t>& vertexTable = bakerOutput.vertexTable;
    std::vector<uint8_t>& dgfBlocks = bakerOutput.dgfBlocks;

    uint32_t vertexOffset = 0;
    for( size_t i=0; i<dgfBlocks.size(); i += DGF::BLOCK_SIZE )
    {
        uint8_t* pBlock = dgfBlocks.data() + i;
        size_t numVerts = DGF::DecodeVertexCount( pBlock );
	
	// Use the DGF 'user-data' field to store each block's offset in the output vertex array
	//  This requires setting the 'enableUserData' field in the baker config.
	DGF::WriteUserData( pBlock, &vertexOffset, 0, sizeof(vertexOffset));
	
	// build the duplicated attribute array.
        for( size_t j=0; j<numVerts; j++ )
        {
            uint32_t inputIndex = vertexTable[vertexOffset++];
            outputAttributes.push_back( inputAttributes[inputIndex] );
        }
    }
}
```

The memory overhead of this vertex duplication depends on the DGF encoding density and the behavior of the content.  For large vertices it can be more efficient to store a duplicated index into the original, deduplicated vertex data.  This reduces the deduplication cost in exchange for an additional indirection when loading the data.  

The table below shows the measured memory overheads for several of the options, measured in bytes per triangle.  This may be compared with the fixed cost of an index buffer (3-12B/Tri), and the size of the DGF data itself (typically 3-6B/Tri).
	
| DGF Target Bitrate | Indexed (2B/Vertex)   | Indexed (4B/Vertex) | Duplicated (8B/Vertex) | Duplicated (16B/Vertex) | Duplicated (32B/Vertex) |
|-------------------:|-----------------------:|---------------------:|-------------------:|--------------------:|--------------------:|
| 11	             | 1.57	                  | 3.14	             | 2.24               |	4.48	            |  8.95			      |
| 12	             | 1.58	                  | 3.16	             | 2.27               |	4.55	            |  9.10			      |
| 13	             | 1.60	                  | 3.19	             | 2.34               |	4.69	            |  9.37			      |
| 14	             | 1.62	                  | 3.24	             | 2.45               |	4.89	            |  9.78			      |
| 15	             | 1.66	                  | 3.31	             | 2.58               |	5.17	            |  10.33		      |
| 16	             | 1.69	                  | 3.38	             | 2.72               |	5.43	            |  10.86		      |
| 24	             | 1.84	                  | 3.69	             | 3.34               |	6.68	            |  13.36		      |


## Using DGFTester

The 'DGFTester' tool can be used to encode OBJ or PLY models and verify that the encoding is correct.  It's syntax is:

```
USAGE: DGFTester.exe <infile> [OPTIONS]
    <infile> is a wavefront obj or ply file
    OPTIONS are:
         --cluster-max-faces <uint>
         --cluster-max-vertices <uint>
         --target-bits <uint>
         --print-perf
         --skip-validation
         --dump-obj <path>
         --dump-bin <path>
         --write-stats <path>
         --discard-materials
         --measure-error
         --forced-offset-width <uint> <uint> <uint>
         --user-data
         --packer {HPG24|SAH}
```		 

By default the tool will compress the input model and print compression statistics.  It will also validate the decoded geometry to ensure that it matches the input, and that the quantized positions are as expected.

The `--dump-obj` option may be used to decode the compressed geometry and write a corresponding obj file.  

The `--print-perf` option may be used to profile the baking process.

## Decoding DGF in HLSL

The SDK provides an HLSL utility library for decoding DGF data on GPUs.  The first step in this process is to load the block header and parse the compression meta-data.   This is accomplished using the `DGFLoadBlockInfo` function, whose signature is shown below:

```
DGFBlockInfo DGFLoadBlockInfo(ByteAddressBuffer dgfBuffer, in uint dgfBlockIndex);
```

The next step is to fetch the vertex indices for a particular triangle in the block.  This returns the block-local vertex indices for that triangle.  There are two functions available for doing this:

```
uint3 DGFGetTriangle_BitScan_Wave(DGFBlockInfo s, uint triangleIndexInBlock)
uint3 DGFGetTriangle_BitScan_Lane(DGFBlockInfo s, uint triangleIndexInBlock)
```

The `_Wave` version assumes that it is running a full wave in uniform control flow, and that the DGFBlockInfo structure is uniform across the wave.  This permits the use of wave intrinsics for a slightly faster implementation.   The `_Lane` version is a single-lane alternative which makes no assumptions about wave structure.

After obtaining the indices, the vertices of the triangle may be fetched from the DGF block using the local indices using `DGFGetVertex`:

```
float3 DGFGetVertex(DGFBlockInfo s, uint vertexIndex);
```

The code example below shows how these functions may be used together to calculate a normal vector for a compressed triangle:

```
float3 CalculateDGFNormal( ByteAddressBuffer dgfBlockBuffer, uint dgfBlockIndex, uint triangleIndexInBlock )
{
    DGFBlockInfo dgfBlock = DGFLoadBlockInfo(dgfBlockBuffer, dgfBlockIndex);
    uint3 localIndices = DGFGetTriangle_BitScan_Lane(dgfBlock, triangleIndexInBlock);
    float3 V0 = DGFGetVertex(dgfBlock, localIndices.x);
    float3 V1 = DGFGetVertex(dgfBlock, localIndices.y);
    float3 V2 = DGFGetVertex(dgfBlock, localIndices.z);                
    return normalize(cross(V1 - V0, V2 - V0));
}
```

