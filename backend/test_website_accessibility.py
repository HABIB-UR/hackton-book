import pytest
from .website_validator import validate_website

def test_validate_website_valid():
    """Test that a valid website URL passes validation."""
    # Using a well-known website for testing
    result, error = validate_website("https://httpbin.org/status/200")
    assert result is True
    assert error is None

def test_validate_website_invalid():
    """Test that an invalid website URL fails validation."""
    result, error = validate_website("https://invalid-url-that-does-not-exist-12345.com")
    assert result is False
    assert error is not None
    assert "not accessible" in error.lower()

def test_validate_website_malformed():
    """Test that a malformed URL fails validation."""
    result, error = validate_website("not-a-valid-url")
    assert result is False
    assert error is not None
    assert "invalid url format" in error.lower()

if __name__ == "__main__":
    pytest.main([__file__])