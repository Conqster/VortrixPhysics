#pragma once

#include <string>
#include "TextureParemeter.h"

#include "SampleFramework/Display/ApplicationWindow.h"

/// Image data
/// Store/cache image data i.e from loaded data, gpu texture creation, physics 
/// @param T pixels format could be in unsigned char, unsigned short or float
struct ImageData
{
	void* pixels = nullptr;

	uint32 width = 0;
	uint32 height = 0;
	
	PixelFormat pxFormat = PixelFormat::Default;
	PixelType pxType = PixelType::UByte;

	bool IsValid() const
	{
		return pixels != nullptr &&
			width > 0 && height > 0;
	}
};


/// Keeping agnostic
class TextureLoader
{
public: 
	
	static ImageData LoadFromFile(const std::string& path, bool flip, TextureFormat& io_desired_texture) //default RGBA
	{
		int width = 0, height = 0, channels = 0;
		void* buffer = nullptr;

		/// note Default is ised as backend support
		/// default = 0, help force backend like stb_img
		/// so desired channel count 0, then system detects 
		/// img file data channels out. 
		PixelFormat px_format = GetPixelFormat(io_desired_texture);
		int desired_channels = PixelFormatChannelCount(px_format);
		PixelType px_type = PixelTypeFromFormat(io_desired_texture);


		using Load_Data_Func = void*(*)(const char*, int*, int*, int*, bool, int);
		static const Load_Data_Func load_data_table[3] =
		{
			&TextureLoader::LoadData<PixelType::UByte>,
			&TextureLoader::LoadData<PixelType::Float>,
			&TextureLoader::LoadData<PixelType::UShort>
		};

		buffer = (load_data_table[(int)px_type])(path.c_str(), &width, &height, &channels, flip, desired_channels);

		if (!buffer)
				return {};

		px_format = static_cast<PixelFormat>(channels);
		if (IsDefault(io_desired_texture))
			io_desired_texture = DeduceTextureFormat(px_format, px_type);
		
		return { buffer, static_cast<uint32>(width), static_cast<uint32>(height), px_format, px_type };
	}


	static void Free(ImageData& img)
	{
		if (img.pixels)
			FreeBuffer(img.pixels);
		img = {};
	}

private:
	template<PixelType Type>
	static void* LoadData(const char* path, int* o_width, int* o_height, int* o_channel, bool flip = true, int desired_channels = 4);

	static void FreeBuffer(void* buff);
};
/// prevent explicit duplicate instaniation for linker
extern template void* TextureLoader::LoadData<PixelType::UByte>(const char*, int*, int*, int*, bool, int);
extern template void* TextureLoader::LoadData<PixelType::Float>(const char*, int*, int*, int*, bool, int);
extern template void* TextureLoader::LoadData<PixelType::UShort>(const char*, int*, int*, int*, bool, int);


#include "ResourceRegister.h"
class Texture : public vx::NonCopyable
{
public:
	Texture()
	{
		mRegisterIdx = TextureRegistry::Instance().Register(this);
	}
	~Texture()
	{
		TextureRegistry::Instance().Unregister(this);
	}

	bool CreateFromPixel(const void* data, const TextureCreateInfo& params);
	
	/// later store width, height in TextureCreateInfo
	bool CreateRenderTarget(const TextureCreateInfo& params);
	bool CreateRenderTarget(uint32_t width, uint32_t height)
	{
		return CreateRenderTarget({ width, height });
	}
	/// params determines the count
	static bool CreateRenderTargets(Texture textures[], const std::vector<TextureCreateInfo>& params);

	bool Resize(uint32_t width, uint32_t height);

	bool Bind(unsigned int slot = 0) const;
	void Unbind() const;

	uint32 ID() const { return mID; }
	void GenerateBindless()
	{
		VX_ASSERT_WARN_VOID(ApplicationWindow::SupportsBindless() && mID !=0 , "Texture is null or does not support bindless");

		mBindlessHandle = glGetTextureHandleARB(mID);
		glMakeTextureHandleResidentARB(mBindlessHandle);
	}

	uint64_t BindlessHandle()
	{
		if (mBindlessHandle == 0)
			GenerateBindless();


		return mBindlessHandle;
	}
	uint32 Width() const { return mWidth; }
	uint32 Height() const { return mHeight; }
	TextureFormat InternalFormat() const { return mInternalFormat; }

	inline bool IsValid() const { return mID != 0; }
	void SetDebugName(const std::string& name)
	{
		VX_ASSERT(mRegisterIdx != TextureRegistry::kInvalidIdx);
		TextureRegistry::Instance().SetAssetName(mRegisterIdx, name);
		ApplyGPUDebugName(name);
	}
	std::string_view DebugName() const { VX_ASSERT(mRegisterIdx != TextureRegistry::kInvalidIdx);  return TextureRegistry::Instance().GetAssetName(mRegisterIdx); }

	void Destroy();

private:
	PixelFormat PixelFormat() const { return GetPixelFormat(mInternalFormat); }
	PixelType PixelType() const { return PixelTypeFromFormat(mInternalFormat); }
	void ApplyGPUDebugName(const std::string& name);

private:
	friend TextureRegistry;
	unsigned int mID = 0;
	TextureRegistry::Idx mRegisterIdx = TextureRegistry::kInvalidIdx;
	uint32 mWidth = 0;
	uint32 mHeight = 0;

	uint64_t mBindlessHandle = 0;
	TextureFormat mInternalFormat = TextureFormat::RGBA8;
	bool bMipmaped = false;
};



class Sampler : vx::NonCopyable
{
public:
	Sampler() = default;
	Sampler(const SamplerCreateInfo& sampler_ci)
	{
		Create(sampler_ci);
	}
	~Sampler() = default;

	bool Create(const SamplerCreateInfo& sampler_ci);

	void Bind(unsigned int slot = 0);
	void Unbind();

	uint32 ID() const { return mID; }
	inline bool IsValid() const { return mID != 0; }

	void Destroy();
private:
	unsigned int mID = 0;

	TextureWrap mWrapModeS = TextureWrap::Repeat;
	TextureWrap mWrapModeT = TextureWrap::Repeat;
	TextureFilter mMinFilter = TextureFilter::Linear;
	TextureFilter mMagFilter = TextureFilter::Linear;
	float mBorderColour[4] = { 1, 1, 1, 1 };
};


struct BindlessTextureSampler : vx::NonCopyable
{
	uint64_t mHandle = 0;
	BindlessTextureSampler(const Texture& texture, const Sampler& sampler)
	{
		mHandle = BindlessTextureSampler::Generate(texture, sampler);
	}

	uint64_t static Generate(const Texture& texture, const Sampler& sampler)
	{
		VX_ASSERT_WARN_RETURN(ApplicationWindow::SupportsBindless(), 0, "Bindless Texture not supported"); 
		VX_ASSERT_WARN_RETURN(texture.IsValid() && sampler.IsValid(), 0, "GL Bindless texture sampler not generate; texture or sampler not valid");

		
		uint64_t handle = glGetTextureSamplerHandleARB(texture.ID(), sampler.ID());
		if (glIsTextureHandleResidentARB(handle))
		{
			VX_LOG_WARN("Bindless Textsure sample was already created for contexxt; Texture: ",
				texture.ID(), ", Sampler: ", sampler.ID());
			return handle;
		}
		GLCall(glMakeTextureHandleResidentARB(handle));
		return handle;
	}

	operator uint64_t() const { return mHandle; }

	~BindlessTextureSampler() { Destroy(); }

	void Destroy()
	{
		if (mHandle != 0)
		{
			glMakeTextureHandleNonResidentARB(mHandle);
			mHandle = 0;
		}
	}
};

class TextureFactory
{
public:

	/// on fail texture handle is still create, to prevent invalid creation, call needs to use texture.valid to check if a texture is still valid
	/// see other part of texture; (e.g sampler/BindlessTextureSampler), takes texture as ref and check if valid before creatung bindless texture sampler
	/// the texture pointer could be reused to generate texture if the pointer is not ouft of scope
	static vx::Ref<Texture> CreateFromFile(const std::string& path, bool flip, const char* name = nullptr, const bool generate_mipmap = false, 
		 TextureFormat internal_format = TextureFormat::DefaultByte)
	{
		vx::Ref<Texture> tex = vx::MakeRef<Texture>();
		VX_ASSERT_WARN_RETURN(vx::FileExist(path.c_str()), tex, (path + " Texture file does not exist").c_str());

		ImageData img_data = TextureLoader::LoadFromFile(path, flip, internal_format);

		if(img_data.IsValid())
		{
			std::string _name = (name) ? name : "";
			tex->CreateFromPixel(img_data.pixels, { img_data.width, img_data.height, internal_format, img_data.pxFormat, img_data.pxType, generate_mipmap, _name });
		}
		else
		{
			VX_ASSERT_WARN(img_data.pixels, (path + ": Invalid Texture image pixels").c_str());
			VX_ASSERT_WARN(img_data.width > 0 && img_data.height > 0, (path + ": Invalid Texture image size").c_str());
		}

		TextureLoader::Free(img_data);
		return tex;
	}
};