# DGFSDK

DGF (Dense Geometry Format) is a block-based geometry compression technology developed by AMD.  It is a hardware-friendly format which will be directly supported by future AMD GPU Architectures.  For more information, refer to the [technical paper](https://gpuopen.com/download/publications/DGF.pdf).

This repository contains our DGF encoding toolchain.  The directory structure is as follows:

* **DGFLib**:  A library containing the low-level encoder/decoder for DGF blocks
* **DGFBaker**: A library which implements a DGF content baking pipeline
* **DGFTester**: A command-line test harness
* **DGFSample**: A simple D3D12/Vulkan viewer for DGF models, which demonstrates real-time decoding in HLSL shaders. 
* **DGFAnimationSample**: A D3D12/Vulkan application demonstrating on the fly DGF block assembly for animated content.
 
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

# Running the Real-Time Samples

The Real-Time samples (DGFSample and DGFAnimationSample) should build and run in your IDE out of the box, and support both Vulkan and D3D12.  D3D12 is used by default.  To run the samples under Vulkan, add --vulkan to the command line.
When run in Vulkan, the samples will leverage the [Vulkan extension](https://gpuopen.com/learn/dense-geometry-format-amd-vulkan-extension/) where available.

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
    DGFBaker::Config config = {};
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
	    memcpy( &output[3*i], vertexData + vertexStride*index, 3*sizeof(float) );
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
### Clustering Without Compression

The first stage of a DGF compression pipeline is to arrange the triangles into SAH-efficient clusters.  The `DGFBaker` class provides a method which allows the cluster builder to be accessed without going all the way to DGF blocks.  The signature is:

```C
std::vector<BakerMeshCluster> BuildClusters( const BakerMesh& mesh );
```
The `BakerMeshCluster` indicates the vertices and triangles from the input mesh which are included in the cluster, and the local connectivity:

```C
struct BakerMeshCluster
{
    std::vector<uint32_t> VertexIndices;  // index of each cluster vert in the input mesh
    std::vector<uint8_t>  Topology;       // index of each triangle vert in 'VertexIndices' (3 per triangle)
    std::vector<uint32_t> PrimIDs;        // index of each triangle in the input mesh
};
```
The cluster builder will guarantee that the resulting clusters respect the `BakerConfig::clusterMaxFaces` and `BakerConfig::clusterMaxVertices` fields.  If `BakerConfig::weldVertices` is true, vertices are de-duplicated during cluster formation.  The cluster builder will guarantee that no two vertices in `VertexIndices` have the same position.

### Compressing Pre-Clustered Geometry

The `DGFBaker` class also supports applying DGF compression to pre-clustered input.  This enables DGF compression to be applied to triangle clusters built by other popular clustering tools.  The pre-clustered input path uses a `ClusteredMesh` class which is similar in spirit to the `BakerMesh` class:

```C
void ClusteredMeshExample( std::vector<MyCluster>& myClusters )
{
    DGFBaker::ClusteredMesh mesh(
       GetVertexReader(myClusters),
       GetIndexReader(myClusters),
       GetAttributeReader(myClusters) );

    DGFBaker::BakerOutputPreclustered output = baker.BakeDGFPreclustered(clusteredMesh);
}

```
The `DGFBaker::BakerOutputPreclustered` contains separate arrays of DGF blocks and meta-data for each of the input clusters.  
Like the `BakerMesh`, the `ClusteredMesh` class is built around customizable reader functions:

```C
// Reads cluster vertices into an array of 3xNumVerts floats.   
//   Args are:   cluster index, output pointer, max vert count
//   Returns the number of vertices
typedef std::function<size_t(size_t, float*, size_t)> ClusterVertexReader;

// Reads cluster connectivity into an array of 3*NumTris indices.   
//   Args are:   cluster index, output pointer, max triangle count
//   Returns the number of triangles
typedef std::function<size_t(size_t, uint8_t*, size_t)> ClusterIndexReader;

// Reads triangle attributes. 
//  Args are:  cluster index, output ptr, max triangle count
//  Returns the number of triangles
typedef std::function<size_t(size_t, DGFBaker::TriangleAttributes*,size_t)> ClusterAttributeReader;
```

A `DGFBaker::ClusteredBakerMesh` class is also provided to enable routing the output of `DGFBaker::BuildClusters` into `DGFBaker::BakeDGFPreclustered`.   

The following example shows an alternate way to execute a full DGF baking pipeline, which may be more useful if the application wishes to preserve the clustering that the `DGFBaker` produced.

```C
void PreclusteredMeshExample( DGFBaker::Baker& baker, DGFBaker::BakerMesh& mesh )
{
    std::vector<DGFBaker::BakerMeshCluster> clusters = baker.BuildClusters(mesh);
    DGFBaker::ClusteredBakerMesh clusteredMesh = DGFBaker::ClusteredBakerMesh(mesh,clusters);
    DGFBaker::BakerOutputPreclustered output = baker.BakeDGFPreclustered(clusteredMesh);

    // use 'output'
}
```


### Baker Configuration

The DGFBaker uses a simple configuration structure shown below:

```C
struct Config
{
        size_t clusterMaxFaces = 128;
        size_t clusterMaxVerts = 256;
         
        size_t blockMaxTris  = 64;
        size_t blockMaxVerts = 64;
        uint8_t blockForcedOffsetWidth[3] = { 0,0,0 }; 
        std::ostream* outputStream = nullptr;
        bool printPerfData = false;                     
        bool validateClusters = false;                 
        bool generateVertexTable = false;              
        bool generateClusterTable = false;             
        bool generateTriangleRemap = false;            
        bool enableUserData = false;                   
        bool encoderRoundTripValidation = false;               
        PackerMode packer = PackerMode::DEFAULT;

        QuantizationMode quantizationMode = QuantizationMode::DEFAULT;

        // Used only when QuantizationMode == `TARGET_BIT_WIDTH`
        size_t targetBitWidth = 16;
        bool quantizeForAnimation = false;       
        float clusterDeformationPadding = 1.0f;  

        // Used only when QuantizationMode == `EXPLICIT_EXPONENT`
        int8_t quantizationExponent = -15; 

        bool enableExponentAdjust  = true;    
        bool weldVertices = false; 
};
    
```

The algorithms used in DGFBaker are described in the [HPG 2024 Paper](https://gpuopen.com/download/publications/DGF.pdf).  We refer the reader to this paper for additional context.  The fields in the `BakerConfig` structure are documented below.

|                  |                                                                        |
|------------------|------------------------------------------------------------------------|
|  clusterMaxFaces | Maximum number of triangles in a cluster (must be <= 256) |
|  clusterMaxVerts | Maximum number of vertices in a cluster (must be <= 256) |
|  blockMaxTris    | Maximum number of triangles allowed in a block |
|  blockMaxVerts   | Maximum number of vertices allowed in a block |
| blockForcedOffsetWidth | Can be used to lock the bit width of the per-vertex offsets in the block.  If set to a non-zero value, the corresponding coordinate will be assigned at least that many bits.  Additional bits will be assigned if the sum of all coordinate sizes is not a multiple of four.  This field is intended to support in-place vertex updaters for animated blocks   |
| outputStream  | An IOStream object to use for diagnostic output |
| printPerfData | If set, performance profiling information will be printed to `outputStream` | 
| validateClusters | Enables debug checks on cluster construction |
| generateVertexTable | If set, the vertex remapping table in the `BakerOutput` will be populated |
| generateClusterTable | If set, the cluster table in the `BakerOutput` will be populated |
| generateTriangleRemap | If set, the triangle remapping table in the `BakerOutput` will be populated |
| enableUserData | Controls whether to reserve space for a 32-bit user-data field in each block.  Setting this to false improves compression rate. |
| encoderRoundTripValidation | Enables debug checks on the DGF block encoder |
| packer | Selects the block packing algorithm to use.  The packer modes are listed below. |
| quantizationMode | Selects the vertex quantization mode to use.  The quantization modes are listed below.|
| targetBitWidth | This corresponds to the `b` parameter from the HPG 2024 paper.  It is the target bit width for the per-vertex offsets, and controls the tradeoff between compression rate and vertex accuracy.  This field is used only if `QuantizationMode` is `TARGET_BIT_WIDTH` |
| quantizeForAnimation | Enables animation-aware quantization.  Used only if `QuantizationMode` is `TARGET_BIT_WIDTH`|
| clusterDeformationPadding | Multiplier to apply to cluster AABB size to account for size change due to deforming animations.  This is a percentage (e.g. 1.05f is 5%). Used only if `QuantizationMode` is `TARGET_BIT_WIDTH` and animation-aware quantization is enabled |
| quantizationExponent | Fixed exponent to use if `QuantizationMode` is set to `EXPLICIT_EXPONENT` |
| enableExponentAdjust | Allows the baker to adjust the block exponents to eliminate trailing zero bits.  This can significantly improves the compression rate for pre-quantized input. |
| weldVertices | Allow the encoder to merge input vertices with identical positions but different indices. When this option is enabled, the block-level connectivity cannot be used as an implicit index buffer. |

The supported packer modes are:

* `DGFBaker::PackerMode::HPG24`:  The algorithm described in the HPG 2024 paper
* `DGFBaker::PackerMode::SAH`:  An alternate algorithm which performs SAH splits until the resulting triangles fit in a single block.

The `SAH` packer mode is the recommended default.  The `HPG24` algorithm is provided as an alternative, and produces slightly better compression rates, but the resulting blocks have a larger surface area and more vertex duplication, which makes them less suitable for ray tracing or fine-grained culling.

The supported quantization modes are:

* `DGFBaker::QuantizationMode::TARGET_BIT_WIDTH`: Chooses a quantization exponent using the algorithm described in the HPG24 paper.  The `BakerConfig::targetBitWidth` field controls the size/quality tradeoff.  This is the preferred method for static geometry.
* `DGFBaker::QuantizationMode::EXPLICIT_EXPONENT`: The `BakerConfig::quantizationExponent` field is used directly as the vertex quantization factor.  

### Animation-Aware Compression

By default, DGF compression assumes that vertices are static, but this is not strictly required.  In order to support animated vertices, the DGFBaker provides an animation-aware flow.  This is enabled by setting `Config::quantizeForAnimation` to true.  Animation-aware compression can result in reduced accuracy and worse compression rates, and should not be enabled for purely static geometry.

When the animation-aware compression is enabled, a more conservative vertex quantization is used in order to prevent overflow during animation:
* The quantization factor is selected based on the diagonal of cluster's AABB, to allow for rotation
* A deformation padding factor is applied to cluster AABB size to allow for size change due to deformation.
* To prevent anchor overflow for large coordinate values, an optional "animation extent" is used to select the quantization factor to prevent anchor overflow.

Although not strictly required, the `blockForcedOffsetWidth` field will usually be used to reserve a fixed amount of encoding space for animated vertices, and allow for efficient in-place block updates.

The following example demonstrates an animation-aware compression flow:

```C
DGFBaker::BakerOutput BakeDGF( const float* vertices, const uint32_t* indices, size_t numVerts, size_t numTris )
{
    DGFBaker::Config config = {};
    config.quantizeForAnimation = true;
    config.clusterDeformationPadding = 1.03f; // pad cluster AABBs by 3%
    config.blockForcedOffsetWidth[0] = 12;  // assign a fixed precision level for the vertex data
    config.blockForcedOffsetWidth[1] = 12;
    config.blockForcedOffsetWidth[2] = 12;
    config.generateVertexTable = true; // generate a table containing the input index for each DGF block vertex
    config.enableUserData = true; // allocate a 32b user-data field in each block to hold its vertex table location
    DGFBaker::Baker baker(config);
    
    DGFBaker::BakerMesh mesh(vertices, indices, numVerts, numTris);

    DGFBaker::AnimationExtents extents = {};
    float extentMin[3] = ...; // compute animation bounding box
    float extentMax[3] = ...; // (NOT SHOWN)
    extents.Set(extentMin[0],extentMin[1],extentMin[2],
                extentMax[0],extentMax[1],extentMax[2]);

    // add the animation extents to the baker mesh
    mesh.SetAnimationExtents(extents);

    // perform the bake
    return baker.BakeDGF(mesh);
}
```

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

