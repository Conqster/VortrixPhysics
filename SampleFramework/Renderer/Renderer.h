#pragma once



#include "SampleFramework/Display/ApplicationWindow.h"

#include "Shader.h"
#include "GPUVertexData.h"

#include <vector>
#include <array>

#include "Framebuffer.h"

#include "ResourceRegister.h"

#include "DebugGizmosRenderer.h"

#include "Vortrix/Core/StackString.h"

///TO-DO(JAY):  this definqtely need to be cleaned 
#include <string>
#include <fstream>
#include <sstream>

//need Window resize listener 

//struct Light
//{
//	vx::Vec3 direction;
//	vx::Vec3 colour;
//};
#include "Vortrix/Visuals/Renderers.h"
#include "Vortrix/Visuals/ERenderInstanceFlags.h"

#include <Vortrix/Core/Colours.h>


#include "Font.h"

#include "GraphicsBuffer.h"

class Texture;
class Sampler;
class Camera;
class RendererImpl : public vx::Renderer
{
public:
	RendererImpl() = default;
	~RendererImpl() = default;

	RendererImpl(ApplicationWindow* display_window);
	void Initialise(ApplicationWindow* display_window);

	//This is just test renderable entity functionality 
	//Helpers
	/// 
	void SubmitSpherePrimitive(const vx::RenderableEntity entity, const vx::ERenderInstanceFlags flags) override 
	{
		DrawGeometry(entity.transform, entity.colour, mSphereGeometry, (entity.plainTexture) ? mPlainTexSamplerBindless : mCheckersTexSamplerBindless, flags);
	}
	void SubmitCubePrimitive(const vx::RenderableEntity entity, const vx::ERenderInstanceFlags flags) override
	{
		DrawGeometry(entity.transform, entity.colour, mBoxGeometry, (entity.plainTexture) ? mPlainTexSamplerBindless : mCheckersTexSamplerBindless, flags);
	}
	void SubmitQuadPrimitive(const vx::RenderableEntity entity, const vx::ERenderInstanceFlags flags) override
	{
		//if(mUseNewRendering)
			DrawGeometry(entity.transform, entity.colour, mQuadGeometry, (entity.plainTexture) ? mPlainTexSamplerBindless : mCheckersTexSamplerBindless, flags);
		//else
			//AddFrameRenderableEntity({&mQuadPrimitive, entity.transform, entity.solidRender, entity.canCastShadow, entity.colour, entity.plainTexture});
	}
	void SubmitQuadXZPrimitive(const vx::RenderableEntity entity, const vx::ERenderInstanceFlags flags) override
	{
		//if(mUseNewRendering)
			DrawGeometry(entity.transform, entity.colour, mQuadXZGeometry, (entity.plainTexture) ? mPlainTexSamplerBindless : mCheckersTexSamplerBindless, flags);
		//else
//AddFrameRenderableEntity({ &mQuadXZPrimitive, entity.transform, entity.solidRender, entity.canCastShadow, entity.colour, entity.plainTexture });
	}

	void SubmitQuadXZDoubledSidedPrimitive(const vx::RenderableEntity entity, const vx::ERenderInstanceFlags flags) override
	{
		DrawGeometry(entity.transform, entity.colour, 
			mQuadXZDoubledSidedGeometry, 
			(entity.plainTexture) ? mPlainTexSamplerBindless : mCheckersTexSamplerBindless, 
			flags, true);
	}
	void SubmitCapsulePrimitive(const vx::RenderableEntity entity, const vx::ERenderInstanceFlags flags) override
	{
		//if(mUseNewRendering)
			DrawGeometry(entity.transform, entity.colour, mCapsuleGeometry, (entity.plainTexture) ? mPlainTexSamplerBindless : mCheckersTexSamplerBindless, flags);
		//else
			//AddFrameRenderableEntity({ &mCapsulePrimitive, entity.transform, entity.solidRender, entity.canCastShadow, entity.colour, entity.plainTexture });
	}

	void DrawText3D(const std::string_view& text,
		const vx::Vec3& pos, float scale,
		const vx::Colour& col,
		ETextAlignment align = ETextAlignment::Left) override;

	void DrawText3D_DynScale(const std::string_view& text,
		const vx::Vec3& pos, float scale,
		const vx::Colour& col,
		ETextAlignment align = ETextAlignment::Left) override;

	DirectionalLight& GetDirectionalLight() { return mDirLight; }
	void SetDirectionalLight(const DirectionalLight& light) { mDirLight = light; }
	vx::Vec3& GetDirectionalLightDir() { return mDirLight.direction; }
	void SetDirectionalLightDir(const vx::Vec3& dir) { mDirLight.direction = dir; }
	vx::Colour GetDirectionalLightColour() const { return mDirLight.colour; }
	void SetDirectionalLightColour(const vx::Colour& value) { mDirLight.colour = value; }
	float GetDirectionalLightIntensity() { return mDirLight.intensity; }
	void SetDirectionalLightIntensity(float value) { mDirLight.intensity = value; }

	void BeginFrame(Camera* p_camera, vx::Colour clear_color = vx::Colour(0.0f, 0.0f, 0.7f, 1.0f));
	void ShadowPass();
	void DrawPass();
	void EndFrame();




	void EnableWireframe()
	{
		glPolygonMode(GL_FRONT, GL_LINE);
		glPolygonMode(GL_BACK, GL_LINE);
	}

	void DisableWireframe()
	{
		glPolygonMode(GL_FRONT, GL_FILL);
		glPolygonMode(GL_BACK, GL_FILL);
	}

	void DisableDepth()const { GLCall(glDisable(GL_DEPTH_TEST)); }
	void EnableDepth()const { GLCall(glEnable(GL_DEPTH_TEST)); }
	void DepthWriteMask(bool flag) const { GLCall(glDepthMask(GLboolean(flag))); }


	void OnResize(uint32_t w, uint32_t h)
	{
		//static size
		//mTestRt.Resize(w, h);
		//mDirLightDebugRT.Resize(w, h);
		//VX_ASSERT(false);
	}

	Sampler* GetASampler() { return mLinearRepeatSampler; }

	const std::vector<Texture*>& GetTextures() const { return TextureRegistry::Instance().GetAll(); }
	ShadowData* ShadowDataPtr() { return &mShadowData; }

	auto* GetTestRTPtr() { return &mTestRt; }
	auto* GetmDirLightDebugRTPtr() { return &mDirLightDebugRT; }
	RenderableMesh& GetAQuickQuadPrimitive() { return mQuadPrimitive; }

	//bool UsingNewRendering() const { return mUseNewRendering; }
	//void UseNewRendering(bool v) { mUseNewRendering = v; }

	void Destroy();

	float mTime;

	vx::Ref<Font> mFont;
	vx::Ref<Shader> mFontShader;
private:
	ApplicationWindow* mWindow;
	Camera* mCamera = nullptr; 

	ShadowData mShadowData;

	Shader mShadowShader;
	DirectionalLight mDirLight;

	struct GPUShadowData
	{
		vx::Mat44 viewProj = vx::Mat44(1.0f);
		float zfar;
		vx::Ref<BindlessTextureSampler> textureMap = nullptr;
	};
	GPUShadowData mGPUShadowData{};




	///Callbacks 
	void SetCallbacks();



	void DrawObjects(Shader& shader, bool only_depth = false);

	RenderableMesh mQuadPrimitive;

	vx::Ref<Texture> mBrickTexture = nullptr;
	vx::Ref<Texture> mCheckersTexture = nullptr;
	vx::Ref<Texture> mPlainTexture = nullptr;

	Sampler* mLinearRepeatSampler = nullptr;
	Sampler* mNearestClampBorderSampler = nullptr;

	ShadowMapRenderTarget mShadowMapRT;
	//RenderTarget<Texture, TextureAttachment> mTestRt;
	RenderTarget<TextureRTDesc, RenderbufferRTDesc> mTestRt;
	RenderTarget<TextureRTDesc, RenderbufferRTDesc> mDirLightDebugRT;

	//RenderTarget<TextureRTDesc, NoDepth> mTestRt;



	struct VertexPrimitiveCreateInfo
	{
		uint32_t stride = 0;
		uint32_t verticesCount = 0;
		uint32_t verticesTypeSizeHack; //actually in most case its the stride
		const void* vertexBufferData = nullptr;

		uint32_t indicesCount = 0;
		uint32_t indicesTypeSizeHack; //actually in most case its the stride
		const void* indexBufferData = nullptr;

		BufferMemory memeory = BufferMemory::HostVisible;
		BufferFrequency usage_freq = BufferFrequency::Dynamic;
	};

	struct VertexPrimitive
	{
		unsigned int VAO = 0;
		GraphicsBuffer vertexBuffer;
		GraphicsBuffer indexBuffer;
		uint32_t stride = 0; //<- of now 
		uint32_t count = 0;

		void GenerateVertexBuffer(uint32_t _stride, uint32_t vertice_count, const void* data,
			BufferMemory mem = BufferMemory::HostVisible, BufferFrequency usage_freq = BufferFrequency::Dynamic)
		{
			stride = _stride;
			count = vertice_count;

			glBindVertexArray(0);
			glCreateVertexArrays(1, &VAO);
			glBindVertexArray(VAO);
			vertexBuffer.Generate(BufferUsage::Vertex, stride, count, data, mem, usage_freq);
			glBindVertexArray(0);
		}

		void Generate(const VertexPrimitiveCreateInfo& create_info)
		{
			stride = create_info.stride;
			count = create_info.verticesCount;

			glBindVertexArray(0);
			glCreateVertexArrays(1, &VAO);
			glBindVertexArray(VAO);

			if(create_info.vertexBufferData)
				vertexBuffer.Generate(BufferUsage::Vertex, create_info.verticesTypeSizeHack, count, create_info.vertexBufferData, create_info.memeory, create_info.usage_freq);

			if(create_info.indexBufferData)
				indexBuffer.Generate(BufferUsage::Index, create_info.indicesTypeSizeHack, create_info.indicesCount, create_info.indexBufferData, create_info.memeory, create_info.usage_freq);

			glBindVertexArray(0);
		}

		void BindLayout(const VertexAttribute* attrib, int count)
		{
			vertexBuffer.BindVertexLayout(attrib, count, VAO, stride);
		}

		void BindArray() { GLCall(glBindVertexArray(VAO)); }
		void BindBuffer() { vertexBuffer.Bind(0); }

		void DrawInstances(size_t instance_count)
		{
			BindArray();
			if (indexBuffer.IsValid())
			{
				indexBuffer.Bind(0);
				glDrawElementsInstanced(GL_TRIANGLES, indexBuffer.count, GL_UNSIGNED_INT, (void*)0, instance_count);
			}
			else
				glDrawArraysInstanced(GL_TRIANGLES, 0, count, instance_count);
		}

		void DrawWireFrameInstanced(size_t instance_count)
		{
			BindArray();
			if (indexBuffer.IsValid())
			{
				indexBuffer.Bind(0);
				glDrawElementsInstanced(GL_LINES, indexBuffer.count, GL_UNSIGNED_INT, (void*)0, instance_count);
			}
			else
				glDrawArraysInstanced(GL_LINES, 0, count, instance_count);
		}

		void Draw()
		{
			BindArray();
			if (indexBuffer.IsValid())
			{
				indexBuffer.Bind(0);
				glDrawElements(GL_TRIANGLES, indexBuffer.count, GL_UNSIGNED_INT, (void*)0);
			}
			else
				glDrawArrays(GL_TRIANGLES, 0, count);
		}
		
		void Destroy()
		{
			vertexBuffer.Destroy();
			glDeleteVertexArrays(1, &VAO);
		}
	};



	using VertexBatch = Ref<VertexPrimitive>;
	class Geometry : vx::NonCopyable
	{
	public:
		Geometry(const VertexBatch& ref_vertices_batch) : mBatch(ref_vertices_batch) {}

		const VertexBatch GetBatch() const { return mBatch; }

		vx::Ref<Geometry> Instance()
		{
			return vx::MakeRef<Geometry>(mBatch);
		}

		void AssignGPULabel(const char* name)
		{
			//int count = name.size() + 4;
			if (mBatch->VAO)
			{
				vx::StackString<32> _name;
				_name << name << " VAO";
				glObjectLabel(GL_VERTEX_ARRAY, mBatch->VAO, _name.Length(), _name.Data());
			}
			if (mBatch->vertexBuffer.IsValid())
				mBatch->vertexBuffer.AssignGPULabel(name);
			if (mBatch->indexBuffer.IsValid())
				mBatch->indexBuffer.AssignGPULabel(name);

		}

	private:
		VertexBatch mBatch = {};
	};

	struct Vertex
	{
		vx::Float3	position{ 0.0f };
		vx::Float3	normal{ 1.0f };
		vx::Vec2	uv{ 0.5f };
		vx::Colour	colour = vx::Colour::sMagenta;
	};

	struct Triangle
	{
		Vertex v0;
		Vertex v1;
		Vertex v2;
	};

	VertexBatch CreateTriangleBatch(const Triangle* triangle, int count)
	{
		//get triangle shadera
		VertexBatch vertex_batch = vx::MakeRef<VertexPrimitive>();
		//vertex_batch->Generate(sizeof(DebugGizmosRenderer::Triangle) / 3, count * 3, triangle, GL_STATIC_DRAW/*, vx::MakeRef<Shader>(mShader)*/);
		vertex_batch->GenerateVertexBuffer(sizeof(Vertex), count * 3, triangle, BufferMemory::DeviceLocal/*, vx::MakeRef<Shader>(mShader)*/);

		std::array<VertexAttribute, 4> attributes =
		{ {
			{0, 3, VertexAttribute::Type::Float, false, offsetof(Vertex, position)},
			{1, 3, VertexAttribute::Type::Float, false, offsetof(Vertex, normal)},
			{2, 2, VertexAttribute::Type::Float, false, offsetof(Vertex, uv)},
			{3, 4, VertexAttribute::Type::UInt8, true, offsetof(Vertex, colour)},
		} };

		//vertex_batch->BindLayout(attributes, 3);
		vertex_batch->BindLayout(attributes.data(), attributes.size());
		return vertex_batch;
	}

	VertexBatch CreateTriangleBatch(const Vertex* vertices, int count, uint32_t* indices, size_t indices_count)
	{
		//get triangle shadera
		VertexBatch vertex_batch = vx::MakeRef<VertexPrimitive>();


		if (indices)
		{
			VertexPrimitiveCreateInfo ci; 
			ci.stride = sizeof(Vertex);
			ci.verticesCount = count;
			ci.verticesTypeSizeHack = sizeof(Vertex);
			ci.vertexBufferData = vertices;

			ci.indicesCount = indices_count;
			ci.indicesTypeSizeHack = sizeof(uint32_t);
			ci.indexBufferData = indices;

			ci.memeory = BufferMemory::DeviceLocal;

			vertex_batch->Generate(ci);
		}
		else
			vertex_batch->GenerateVertexBuffer(sizeof(Vertex), count, vertices, BufferMemory::DeviceLocal/*, vx::MakeRef<Shader>(mShader)*/);


		std::array<VertexAttribute, 4> attributes = 
		{ {
			{0, 3, VertexAttribute::Type::Float, false, offsetof(Vertex, position)},
			{1, 3, VertexAttribute::Type::Float, false, offsetof(Vertex, normal)},
			{2, 2, VertexAttribute::Type::Float, false, offsetof(Vertex, uv)},
			{3, 4, VertexAttribute::Type::UInt8, true, offsetof(Vertex, colour)},
		} };

		vertex_batch->BindLayout(attributes.data(), attributes.size());
		return vertex_batch;
	}


	struct Instance
	{
		vx::Mat44 matrix;
		vx::Mat44 invMatrix;
		vx::Vec4 colour;
		uint64_t texHandle;
		/// > 0 
		/// in shadow vert shadow, castes shadow 
		/// > 1 
		/// in model shading, receives shadow
		ERenderInstanceFlags flags = ERenderInstanceFlags::CastShadow | ERenderInstanceFlags::ReceiveShadow;
		uint32_t padding;
	};

	struct Instances
	{
		std::vector<Instance>	buffer = {};
		bool					isDirty = false;
		bool					disableCulling = false;
		bool					wireFrame = false;
	};
	/// bucket/buffer accompany with UBO
	/// for easy instance GPU dump
	//using InstanceBucket = std::vector<Instance>;
	//std::unordered_map<Ref<Geometry>, InstanceBucket> mSolidGeometries;
	/// probaly make key unique (not geometry point) later to support same geometry 
	/// data but customed; culled/not culled etc
	std::unordered_map<Ref<Geometry>, Instances> mSolidGeometries;

	vx::Ref<Geometry> mBoxGeometry;
	vx::Ref<Geometry> mSphereGeometry;
	vx::Ref<Geometry> mCapsuleGeometry;
	vx::Ref<Geometry> mQuadXZGeometry;
	vx::Ref<Geometry> mQuadXZDoubledSidedGeometry; 
	vx::Ref<Geometry> mQuadGeometry;
	vx::Ref<Shader> mGeometryShader;
	vx::Ref<Shader> mInstanceShadowDepthShader;
	vx::Ref<BindlessTextureSampler> mCheckersTexSamplerBindless;
	vx::Ref<BindlessTextureSampler> mBrickTexSamplerBindless;
	vx::Ref<BindlessTextureSampler> mPlainTexSamplerBindless;
	///might have to move this as part of 
	///Geometry manage there own instance buffer or might 
	/// or might have to create a instance buffer data map
	GraphicsBuffer mInstanceSSBO;
	GraphicsBuffer mCameraUBO;
	GraphicsBuffer mDirLightUBO;
	GraphicsBuffer mDirLightShadowUBO;

	/// bool disable_culling = false; its is for all ionstance group might fix
	void DrawGeometry(const vx::Mat44& matrix, const Colour& col, const Ref<Geometry>& ref_geometry, 
		const Ref<BindlessTextureSampler>& bindless_tex_sampler, 
		ERenderInstanceFlags flags = ERenderInstanceFlags::CastShadow | ERenderInstanceFlags::ReceiveShadow, bool disable_culling = false)
	{
		/////bind appropriate shader/pipweline
		/////
		//auto ref_tri_batch = ref_geometry->GetBatch();
		//ref_tri_batch->BindArray();
		////no need to bind the buffer only is write/upload is needed
		//glDrawArrays(GL_TRIANGLES, 0, ref_tri_batch->count);
		
		//int nxt_idx = mSolidGeometries[ref_geometry].buffer.size();
		//uint64_t use_tex = (nxt_idx%2 == 0) ? *mMeladyTexSamplerBindless : *mCheckersTexSamplerBindless;
		uint64_t use_tex = *bindless_tex_sampler;
		bool _new = mSolidGeometries.find(ref_geometry) == mSolidGeometries.end();
		if (_new)
			mSolidGeometries[ref_geometry].buffer.reserve(512);
		mSolidGeometries[ref_geometry].buffer.push_back({ matrix, matrix.Inverse(), col,  use_tex, flags});
		mSolidGeometries[ref_geometry].isDirty = true;
		mSolidGeometries[ref_geometry].disableCulling = disable_culling;
		mSolidGeometries[ref_geometry].wireFrame = (int(flags & vx::ERenderInstanceFlags::Wireframe) != 0);
		//mSolidGeometries[ref_geometry].push_back({ matrix, matrix.Inverse(), col });
	}


	struct GPULight
	{
		GPULight() = default;
		GPULight(const vx::Float3& dir, float amb_ins = 0.2f, 
			const vx::Float3& col = vx::Float3(), float diff_ins = 0.985f) : 
			direction(dir), ambientIntensity(amb_ins), 
			colour(col), diffuseIntensity(diff_ins){ }

		vx::Float3 direction;
		float ambientIntensity = 0.7f;
		vx::Float3 colour = vx::Float3();
		float diffuseIntensity = 1.0f;


		bool operator ==(const GPULight& rhs) const
		{
			return direction == rhs.direction &&
				ambientIntensity == rhs.ambientIntensity &&
				colour == rhs.colour &&
				diffuseIntensity == rhs.diffuseIntensity;
		}
		bool operator !=(const GPULight& rhs) const { return !(*this == rhs); }
	};

	void RenderGeometriesInstances(bool only_depth = false);
	void ClearGeometriesInstances()
	{
		for (auto& [ref_geometry, instances] : mSolidGeometries)
		{
			instances.buffer.clear();
		}
	}

	void GenerateSphereGeometry()
	{
		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;

		uint32_t sector_count = 36;
		uint32_t span_count = 18;

		//vextex position
		float x, y, z, w;
		//vectex texture coord
		float u, v;

		float sector_angle;
		float span_angle;

		//TO-DO: for now use float, double will be too redundant to use 
		float sector_step = (float)(vx::kVxTau / sector_count);  // 0 - 360(2pi)/count =>  angle btw each steps
		float span_step = (float)(vx::kVxPi / span_count);		  // 0 - 180(pi)/count => angle btw each step

		//float radius = 1.0f;
		float radius = 0.5f;

		//compute & store vertices
		for (unsigned int i = 0; i <= span_count; i++)
		{
			// 180 degree(pi) to 0 degree //0 degree to 180 degree(pi)
			span_angle = (float)vx::kVxPi - i * span_step;

			//parametric equation for sphere
			// x = center_x + r * sin(theta) * cos(phi)    
			// y = center_y + r * sin(thata) * sin(phi)
			// z = center_z + r * cos(theta)
			// where 
			//		theta = span_angle range 0 to pi(180 degrees)
			//		phi = sector_angle range 0 to 2pi(360 degrees)
			//RETERIVED: https://en.m.wikipedia.org/wiki/Sphere
			//			In their case z is up axis


			w = radius * vx::VxSin(span_angle);
			y = radius * vx::VxCos(span_angle);

			//add (sector_count + 1) vertices per stack
			//first and last vertices have same position, but different tex coords 
			for (unsigned int j = 0; j <= sector_count; ++j)
			{
				//from from 0 >> current step * step angle >> 360 
				sector_angle = j * sector_step;

				//vectex position (x, y, z)
				x = w * vx::VxCos(sector_angle);			//r * cos(u) * cos(v)
				z = w * vx::VxSin(sector_angle);			//r * cos(u) * sin(v)

				//vertex texture coord range between [0, 1]
				u = (float)j / sector_count;
				v = (float)i / span_count;

				Vertex vertex
				{
					vx::Float3(x,y, z),
					vx::Float3(x,y, z),
					vx::Vec2(u,v),
					vx::Colour(x,y, z)
				};
				vertices.push_back(vertex);
			}
		}


		//compute & store indices
		unsigned int k1, k2;
		for (unsigned int i = 0; i < span_count; ++i)
		{
			k1 = i * (sector_count + 1);		//beginning of current stack
			k2 = k1 + sector_count + 1;			//beginning of next stack

			for (unsigned int j = 0; j < sector_count; ++j, ++k1, ++k2)
			{
				//2 triangles per sector excluding first and last stacks
				//k1 => k2 => k1+1
				if (i != 0)
				{
					indices.push_back(k1);
					indices.push_back(k2);
					indices.push_back(k1 + 1);
				}

				//k1+1 => k2 => k2+ 1
				if (i != (span_count - 1))
				{
					indices.push_back(k1 + 1);
					indices.push_back(k2);
					indices.push_back(k2 + 1);
				}

			}
		}

		mSphereGeometry = vx::MakeRef<Geometry>(CreateTriangleBatch(vertices.data(), 
			vertices.size(), 
			indices.data(), 
			indices.size()));

		glObjectLabel(GL_VERTEX_ARRAY, mSphereGeometry->GetBatch()->VAO, 10, "Sphere VAO");
		glObjectLabel(GL_BUFFER, mSphereGeometry->GetBatch()->vertexBuffer.iD, 10, "Sphere VBO");
		glObjectLabel(GL_BUFFER, mSphereGeometry->GetBatch()->indexBuffer.iD, 10, "Sphere EBO");

	}




	void CreateSphereOctant(float radius, int resolution, int signX, int signY, int signZ, std::vector<Vertex>& vertices, std::vector<uint32_t>& indices)
	{

		const uint32_t base_vertex = (uint32_t)vertices.size();
		const int stride = resolution + 1;

		vx::Vec3 axes[3] = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
		for(int a = 0; a<3; a++)
		{
			vx::Vec3 normal = axes[a];

			vx::Vec3 tang = axes[(a + 1) % 3];
			vx::Vec3 bitan = axes[(a + 2) % 3];
			for (int y = 0; y <= resolution; y++)
			{
				float fy = (float)y / resolution;
				for (int x = 0; x <= resolution; x++)
				{
					float fx = (float)x / resolution;

					//vx::Vec3 dir = { cubeX, cubeY, 1.0f };
					vx::Vec3 dir = (normal * 1.0f) + (tang * fx) + (bitan * fy);
					dir *= vx::Vec3(signX, signY, signZ);
					//vx::Vec3 dir = vx::Vec3((float)signX * fx, (float)signY * fy, (float)signZ);
					dir.Normalise();


					vx::Float3 nor = dir.ToFloat3();
					vx::Float3 pos = (dir * radius).ToFloat3();


					float longtitude = vx::VxAtan2(nor.y, nor.x);
					float latitude = vx::VxAsin(nor.z);

					vx::Vec2 uv(0.5f + longtitude / (2.0f * vx::kVxPi),
						0.5f - latitude / vx::kVxPi);


					float u = 0.5f + (vx::VxAtan2(nor.z, nor.x) / (2.0f * vx::kVxPi));
					//float u = 0.5f + (atan2(n.z, n.x) / (2.0f * (float)M_PI));
					//latitude asin range [-PI/2, PI/2] map [0, 1]
					float _v = 0.5f + (vx::VxAsin(nor.y) / vx::kVxPi);
					//float _v = (v[i].position.y + span_half) / (span_half * 2.0f);

					uv = vx::Vec2(u, _v);
					Vertex vert
					{
						pos,
						nor,
						uv,
						vx::Colour::sWhite
					};
					vertices.push_back(vert);
				}
			}
		}


		for (int y = 0; y < resolution; y++)
		{
			for (int x = 0; x < resolution; x++)
			{
				uint32_t i0 = base_vertex + y * stride + x;
				uint32_t i1 = i0 + 1;
				uint32_t i2 = i0 + stride;
				uint32_t i3 = i2 + 1;

				if (signX * signY * signZ > 0)
				{
					indices.push_back(i0);
					indices.push_back(i2);
					indices.push_back(i1);

					indices.push_back(i1);
					indices.push_back(i2);
					indices.push_back(i3);
				}
				else
				{
					indices.push_back(i0);
					indices.push_back(i1);
					indices.push_back(i2);

					indices.push_back(i1);
					indices.push_back(i3);
					indices.push_back(i2);
				}
			}
		}

	}

	/// Mark deprecated 
	void SubdivideTriangle(std::vector<uint32_t>& indices, std::vector<Vertex>& vertices,
		vx::Vec3 v1, uint32_t& idx1,
		vx::Vec3 v2, uint32_t& idx2,
		vx::Vec3 v3, uint32_t& idx3,
		int level, float radius)
	{
		if (level == 0)
		{
			// Helper to add vertex if it doesn't exist (idx == -1)
			auto addVertex = [&](vx::Vec3 dir, uint32_t& idx) {
				if (idx == 0xffffffff) {
					idx = (uint32_t)vertices.size();
					dir.Normalise();
					vx::Float3 normal = dir.ToFloat3();
					vx::Float3 pos = (dir * radius).ToFloat3();

					// Simple Spherical UVs
					vx::Vec2 uv(0.5f + vx::VxAtan2(normal.y, normal.x) / (2.0f * vx::kVxPi),
						0.5f - vx::VxAsin(normal.z) / vx::kVxPi);

					if (uv[0] < 0.0f) uv[0] += 1.0f;
					if (uv[0] > 1.0f) uv[0] -= 1.0f;

					if (vx::VxAbs(normal.z) > 0.999f)
						uv[0] = 0.5f;

					vertices.push_back({ pos, normal, uv, vx::Colour::sWhite });
				}
				};

			addVertex(v1, idx1);
			addVertex(v2, idx2);
			addVertex(v3, idx3);

			indices.push_back(idx1);
			indices.push_back(idx2);
			indices.push_back(idx3);
		}
		else
		{
			// Find midpoints and project them to the sphere surface
			vx::Vec3 m1 = (v1 + v2).Normalised();
			vx::Vec3 m2 = (v2 + v3).Normalised();
			vx::Vec3 m3 = (v3 + v1).Normalised();

			uint32_t midIdx1 = 0xffffffff;
			uint32_t midIdx2 = 0xffffffff;
			uint32_t midIdx3 = 0xffffffff;

			// Subdivide into 4 smaller triangles
			SubdivideTriangle(indices, vertices, v1, idx1, m1, midIdx1, m3, midIdx3, level - 1, radius);
			SubdivideTriangle(indices, vertices, m1, midIdx1, v2, idx2, m2, midIdx2, level - 1, radius);
			SubdivideTriangle(indices, vertices, m3, midIdx3, m2, midIdx2, v3, idx3, level - 1, radius);
			SubdivideTriangle(indices, vertices, m1, midIdx1, m2, midIdx2, m3, midIdx3, level - 1, radius);
		}
	}



	/// Recursive subdivision of triange on an octant of a sphere (1/8th of a sphere)
	void RecursiveSubdivide(
		const vx::Vec3& v0, const vx::Vec3& v1,
		const vx::Vec3& v2, int depth, 
		const vx::Vec3& offset, std::vector<Vertex>& vertices)
	{
		if (depth == 0)
		{
			const vx::Vec3 pos[] = { v0, v1, v2 };

			float h_half = vx::VxAbs(offset.Y());
			Vertex v[3];
			for (int i = 0; i < 3; i++)
			{
				(pos[i] + offset).Store(v[i].position);
				((pos[i]).Normalised()).Store(v[i].normal);
				 vx::Float3 n = v[i].normal;

				//longtitude atan2 range[-PI, PI] map [0, 1]
				float u = 0.5f + (atan2(n.z, n.x) / (2.0f * (float)vx::kVxPi));
				//latitude asin range [-PI/2, PI/2] map [0, 1]
				float _v = 0.5f + (vx::VxAsin(n.y) / (float)vx::kVxPi);

				v[i].uv = vx::Vec2(u, _v);

				//ensure colour is not garbage
				v[i].colour = vx::Colour::sWhite;
			}


			//fix
			if (vx::VxAbs(v[0].uv.X() - v[1].uv.X()) > 0.5f ||
				vx::VxAbs(v[1].uv.X() - v[2].uv.X()) > 0.5f ||
				vx::VxAbs(v[1].uv.X() - v[0].uv.X()) > 0.5f)
			{
				for (int i = 0; i < 3; i++)
					if (v[i].uv[0] < 0.5f)
						v[i].uv[0] += 1.0f;
			}

			vertices.insert(vertices.end(), v, v + 3);
			//vertices.push_back(v)
			////mVBO.AddVertexData(&v, sizeof(Vertex)*3);
			//mVBO.AddVertexData(&v, sizeof(v));
			//mIndicesCount++;
			return;
		}

		//target radius 
		float radius = v0.Length();
		//mid points
		const vx::Vec3 v01 = (v0 + v1).Normalised() * radius;
		const vx::Vec3 v12 = (v1 + v2).Normalised() * radius;
		const vx::Vec3 v20 = (v2 + v0).Normalised() * radius;

		RecursiveSubdivide(v0, v01, v20, depth - 1, offset, vertices);
		RecursiveSubdivide(v01, v1, v12, depth - 1, offset, vertices);
		RecursiveSubdivide(v20, v12, v2, depth - 1, offset, vertices);
		RecursiveSubdivide(v01, v12, v20, depth - 1, offset, vertices);
	}



	void CreateSphere2(float radius = 0.5f, int subdivision_level = 3)
	{
		std::vector<Vertex> vertices;
		std::vector<uint32_t> indices;

		bool use_old = false;
		if(use_old)
		{
			// The 6 points of an octahedron
			vx::Vec3 p[] = {
				{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
			};

			// Helper to track indices of the base octahedron points
			uint32_t baseIdx[6] = { 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff };

			// Call for the 8 faces of the octahedron
			// Top 4 triangles
			SubdivideTriangle(indices, vertices, p[4], baseIdx[4], p[0], baseIdx[0], p[2], baseIdx[2], subdivision_level, radius);
			SubdivideTriangle(indices, vertices, p[4], baseIdx[4], p[2], baseIdx[2], p[1], baseIdx[1], subdivision_level, radius);
			SubdivideTriangle(indices, vertices, p[4], baseIdx[4], p[1], baseIdx[1], p[3], baseIdx[3], subdivision_level, radius);
			SubdivideTriangle(indices, vertices, p[4], baseIdx[4], p[3], baseIdx[3], p[0], baseIdx[0], subdivision_level, radius);

			// Bottom 4 triangles
			SubdivideTriangle(indices, vertices, p[5], baseIdx[5], p[2], baseIdx[2], p[0], baseIdx[0], subdivision_level, radius);
			SubdivideTriangle(indices, vertices, p[5], baseIdx[5], p[1], baseIdx[1], p[2], baseIdx[2], subdivision_level, radius);
			SubdivideTriangle(indices, vertices, p[5], baseIdx[5], p[3], baseIdx[3], p[1], baseIdx[1], subdivision_level, radius);
			SubdivideTriangle(indices, vertices, p[5], baseIdx[5], p[0], baseIdx[0], p[3], baseIdx[3], subdivision_level, radius);
		}
		else
		{
			vx::Vec3 vertex[6] =
			{
				{0.0f, 0.5f, 0.0f},//Top
				{0.0f, -0.5f, 0.0f},//Botttom
				{0.5f, 0.0f, 0.0f},//Right
				{-0.5f, 0.0f, 0.0f},//Left
				{0.0f, 0.0f, 0.5f},//Front
				{0.0f, 0.0f, -0.5f},//Back
			};

			float cylinder_height = 0.0f;
			vx::Vec3 offset{ 0.0f, cylinder_height * 0.5f, 0.0f };


			RecursiveSubdivide(vertex[0], vertex[4], vertex[2], subdivision_level, offset, vertices);
			RecursiveSubdivide(vertex[0], vertex[3], vertex[4], subdivision_level, offset, vertices);
			RecursiveSubdivide(vertex[0], vertex[5], vertex[3], subdivision_level, offset, vertices);
			RecursiveSubdivide(vertex[0], vertex[2], vertex[5], subdivision_level, offset, vertices);

			RecursiveSubdivide(vertex[1], vertex[2], vertex[4], subdivision_level, -offset, vertices);
			RecursiveSubdivide(vertex[1], vertex[4], vertex[3], subdivision_level, -offset, vertices);
			RecursiveSubdivide(vertex[1], vertex[3], vertex[5], subdivision_level, -offset, vertices);
			RecursiveSubdivide(vertex[1], vertex[5], vertex[2], subdivision_level, -offset, vertices);
		}


		mSphereGeometry = vx::MakeRef<Geometry>(CreateTriangleBatch(vertices.data(), vertices.size(), indices.data(), indices.size()));

		///assign debug labels 
		mSphereGeometry->AssignGPULabel("Sphere");
	}


	void CreateCapsuleGeometry(float radius = 0.5f, float cylinder_halfheight = 0.5f, int subdivision_level = 3)
	{
		int capsule_seg = 4 * vx::VxPow(2, subdivision_level);
		int sphere_depth = subdivision_level;
		float cylinder_height = cylinder_halfheight * 2.0f;

		std::vector<Vertex> vertices;

		auto Add_Hemisphere = [&](float y_offset, bool top)
			{
				vx::Vec3 v[6] = 
				{
					vx::Vec3(1, 0, 0),
					vx::Vec3(-1, 0, 0),
					vx::Vec3(0, 1, 0),
					vx::Vec3(0, -1, 0),
					vx::Vec3(0, 0, 1),
					vx::Vec3(0, 0, -1)
				};

				int f[8][3] = {
					{0, 2, 4}, {4, 2, 1},
					{1, 2, 5}, {5, 2, 0},

					{0, 4, 3}, {4, 1, 3},
					{1, 5, 3}, {5, 0, 3}
				};

				vx::Vec3 offset{ 0.0f, y_offset, 0.0f };
				for (int i = 0; i < 8; i++)
				{
					vx::Vec3 a = (v[f[i][0]]).Normalised() * radius;
					vx::Vec3 b = (v[f[i][1]]).Normalised() * radius;
					vx::Vec3 c = (v[f[i][2]]).Normalised() * radius;

					///cull bottom hemisphere for top, top for bottom
					if (top && (a.Y() < 0.0f || b.Y() < 0.0f || c.Y() < 0.0f))continue;
					if (!top && (a.Y() > 0.0f || b.Y() > 0.0f || c.Y() > 0.0f))continue;

					RecursiveSubdivide(a, b, c, sphere_depth, offset, vertices);
				}
			};

		Add_Hemisphere(cylinder_halfheight, true);
		Add_Hemisphere(-cylinder_halfheight, false);

		vx::Colour col = vx::Colour::sWhite;

		for (int i = 0; i < capsule_seg; i++)
		{
			float theta_0 = (2.0f * vx::kVxPi * i / capsule_seg);
			float theta_1 = (2.0f * vx::kVxPi * (i + 1) / capsule_seg);


			vx::Float3 p0{ radius * vx::VxCos(theta_0), -cylinder_height / 2, radius * vx::VxSin(theta_0) };
			vx::Float3 p1{radius * vx::VxCos(theta_1), -cylinder_height / 2, radius * vx::VxSin(theta_1)};
			vx::Float3 p2{radius * vx::VxCos(theta_1), cylinder_height / 2, radius * vx::VxSin(theta_1)};
			vx::Float3 p3{radius * vx::VxCos(theta_0), cylinder_height / 2, radius * vx::VxSin(theta_0)};

			//nor
			vx::Float3 n0, n1, n2, n3;
			(vx::Vec3(p0.x, 0.0f, p0.z)).Normalised().Store(n0);
			(vx::Vec3(p1.x, 0.0f, p1.z)).Normalised().Store(n1);
			(vx::Vec3(p2.x, 0.0f, p2.z)).Normalised().Store(n2);
			(vx::Vec3(p3.x, 0.0f, p3.z)).Normalised().Store(n3);


			float total_height = cylinder_height + (2.0f * radius);
			float v_bottom = radius / total_height;
			float v_top = (radius + cylinder_height) / total_height;
			//uv
			vx::Vec2 uv0(i / (float)capsule_seg, v_bottom);
			vx::Vec2 uv1((i + 1) / (float)capsule_seg, v_bottom);
			vx::Vec2 uv2((i + 1) / (float)capsule_seg, v_top);
			vx::Vec2 uv3(i / (float)capsule_seg, v_top);

			Vertex quad[6] = {
				{p0, n0, uv0, col},
				{p2, n2, uv2, col},
				{p1, n1, uv1, col},

				{p0, n0, uv0, col},
				{p3, n3, uv3, col},
				{p2, n2, uv2, col},
			};


			vertices.insert(vertices.end(), quad, quad + 6);
		}

		mCapsuleGeometry = vx::MakeRef<Geometry>(CreateTriangleBatch(vertices.data(), vertices.size(), nullptr, 0));

		///assign debug labels 
		mCapsuleGeometry->AssignGPULabel("Capsule");
	}


	vx::Ref<Geometry> LoadMeshAsGeometry(const std::string& path)
	{
		std::ifstream file_stream(path, std::ios::in);
		if (!file_stream.is_open())
		{
			//CONSOLE_LOG("[Loader Mesh]: Failed to open file: " << path.c_str());
			return {};
		}

		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;
		int has_colour = 0;
		std::string line;
		while (std::getline(file_stream, line))
		{
			std::istringstream iss(line);
			std::string prefix;
			iss >> prefix;
			//CONSOLE_LOG(prefix.c_str());
			if (prefix == "vcount")
			{
				int c;
				iss >> c;
				vertices.reserve(c);
				//CONSOLE_LOG("vertex count: " << c);
			}
			else if (prefix == "hascolour")
			{
				iss >> has_colour;
				//CONSOLE_LOG("vertex has colour: " << ((has_colour) ? "true" : "false"));
			}
			else if (prefix == "icount")
			{
				int c;
				iss >> c;
				indices.reserve(c);
				//CONSOLE_LOG("indices count: " << c);
			}
			else if (prefix == "v")
			{
				Vertex v;
				iss >> v.position[0] >> v.position[1] >> v.position[2] >>
					v.normal[0] >> v.normal[1] >> v.normal[2] >>
					v.uv[0] >> v.uv[1];
				if (has_colour)
				{
					float r, g, b;
					iss >> r >> g >> b;
					v.colour = vx::Colour(r, g, b);
				}
				else
				{
					v.colour = vx::Colour(v.position[0], v.position[1], v.position[2]);
				}

				//hack override 
				v.colour = vx::Colour::sWhite;
				vertices.push_back(v);
				//CONSOLE_LOG("vertex x: " << v.position[0] << "f, y: " << v.position[1] << "f, z : " << v.colour[0] << "f, r : " << v.colour[1] << "f, g : " << v.colour[1] << "f, b : " << v.colour[2] << "f");
			}
			else if (prefix == "i")
			{
				unsigned int i[3];
				iss >> i[0] >> i[1] >> i[2];
				indices.push_back(i[0]);
				indices.push_back(i[1]);
				indices.push_back(i[2]);
				//CONSOLE_LOG("triange indices: " << i[0] << ", " << i[1] << ", " << i[2]);
			}
		}



		return vx::MakeRef<Geometry>(CreateTriangleBatch(vertices.data(), vertices.size(), indices.data(), indices.size()));
	}

	void GenerateBoxGeometry()
	{

		vx::Colour col = vx::Colour(1.0f, 1.0f, 1.0f);

		vx::AABB aabb(0.5f);

		const std::array<vx::Vec3, 8> corners = aabb.GetCorners();


		static const uint32_t faces[6][4] =
		{
			{0, 3, 2, 1},
			{4, 5, 6, 7},
			{0, 1, 5, 4},
			{3, 7, 6, 2},
			{0, 4, 7, 3},
			{1, 2, 6, 5}
		};

		static const vx::Vec2 quad_uv[4] =
		{
			{0.0f, 0.0f}, //bottom left
			{1.0f, 0.0f}, //bottom right
			{1.0f, 1.0f}, //top right
			{0.0f, 1.0f} // top left
		};

		std::vector<Triangle> tri_list;



		for (int f = 0; f < 6; ++f)
		{
			const uint32_t* v = faces[f];

			vx::Vec3 v0 = corners[v[0]];
			vx::Vec3 v1 = corners[v[1]];
			vx::Vec3 v2 = corners[v[2]];
			vx::Vec3 v3 = corners[v[3]];

			vx::Float3 nor = (v1 - v0).Cross(v2 - v0).Normalise().ToFloat3();
			//vx::Float3 nor = (v2 - v0).Cross(v1 - v0).Normalise().ToFloat3();

			//this is fine since the box is at origin with no offset 
			//if they were offset then the corner difference (aabb size) is needed 
			vx::Vec2 uv00 = vx::Vec2(v0.X(), v0.Y());
			vx::Vec2 uv11 = vx::Vec2(v2.X(), v2.Y());

			Triangle tri0 = {
					{v0.ToFloat3(), nor, quad_uv[0], col},
					{v1.ToFloat3(), nor, quad_uv[1], col},
					{v2.ToFloat3(), nor, quad_uv[2], col},
			};
			Triangle tri1 = {
				{v0.ToFloat3(), nor, quad_uv[0], col},
				{v2.ToFloat3(), nor, quad_uv[2], col},
				{v3.ToFloat3(), nor, quad_uv[3], col},
			};
			tri_list.push_back(tri0);
			tri_list.push_back(tri1);
		}


		mBoxGeometry = vx::MakeRef<Geometry>(CreateTriangleBatch(tri_list.data(), tri_list.size()));

		mCheckersTexSamplerBindless = vx::MakeRef<BindlessTextureSampler>(*mCheckersTexture, *mLinearRepeatSampler);
		mBrickTexSamplerBindless = vx::MakeRef<BindlessTextureSampler>(*mBrickTexture, *mLinearRepeatSampler);
		mPlainTexSamplerBindless = vx::MakeRef<BindlessTextureSampler>(*mPlainTexture, *mLinearRepeatSampler);
		//VX_ASSERT(mWindow == nullptr, "");

		//mark buffer for debuggubng 
		mBoxGeometry->AssignGPULabel("Box");


		//InstanceShadowDepth  mInstanceShaderDepth
		mGeometryShader = vx::MakeRef<Shader>();
		mGeometryShader->Create("primitive-geometry-shading",
			"assets/shaders/debug/TriangleBatchInstance.vert", //vertex shader
			"assets/shaders/debug/TriangleBatchInstance.frag", "", true); //fragment shader

		mInstanceShadowDepthShader = vx::MakeRef<Shader>();
		mInstanceShadowDepthShader->Create("primitive-instance-shadowdepth-shading",
			"assets/shaders/InstanceShadowDepth.vert", //vertex shader
			"assets/shaders/shadowDepth.frag"); //fragment shader

		//mGeometryShader->Bind();
		mCameraUBO.Generate(BufferUsage::Uniform, sizeof(GizmosCameraData) + sizeof(vx::Vec4));///sizeof(vx::Float3) hack 
		mCameraUBO.Bind(0);
		mCameraUBO.AssignGPULabel("Camera");

		mDirLightUBO.Generate(BufferUsage::Uniform, sizeof(GPULight));
		mDirLightUBO.Bind(2);
		mDirLightUBO.AssignGPULabel("Dir Light");

		mDirLightShadowUBO.Generate(BufferUsage::Uniform, sizeof(GPUShadowData));
		mDirLightShadowUBO.Bind(3);
		mDirLightShadowUBO.AssignGPULabel("Dir Light Shadow");

		mGPUShadowData.textureMap = vx::MakeRef<BindlessTextureSampler>(
			*(const Texture*)mShadowMapRT.GetAttachment(0)->GetBuffer(),
			*mNearestClampBorderSampler);



		//create require instance buffer 
		int required_count = 10;
		mInstanceSSBO.Generate(BufferUsage::Storage,
			sizeof(Instance) * required_count,
			nullptr, BufferMemory::HostVisible, BufferFrequency::Dynamic);

		//mInstanceSSBO.BindVertexLayout(nullptr, 0, 0, 23);
		mInstanceSSBO.Bind(1);
		mInstanceSSBO.AssignGPULabel("Instance");
	}
	
};