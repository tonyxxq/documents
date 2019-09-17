# JWT(json  web token)

- 阮一峰关于 JWT 的原理描述得很清楚了

  http://www.ruanyifeng.com/blog/2018/07/json_web_token-tutorial.html

- 为什么要使用 JWT?

  传统的使用 session 的方式，用户信息保存在后台，当后台应用比较多的时候，**跨应用单点登录解决起来比较麻烦**，但是使用 JWT 的方式，不需要服务器存储任何东西（除了 secret）,当用户第一次登录成功之后就把用户信息返回给用户，用户保存在自己本地，下次访问的时候就拿着上次的放在 header 里边进行后台访问，后台对信息进行校验（主要是校验， 使用在服务器端存储的 secret 对用户传的数据加密后生成的 signature 和用户传递的 signature 是否一致），访问别的系统也是一样，但是需要保证各个系统的 secret 需要一致

  **注意：注意不要在 jwt 响应内容里边存密码等信息，因为用户是可以解密的**

- JWT 的缺点

  JWT 只设置了有效时间，且保存在 jwt 的文本信息中，如果有效时间没有到，服务端是不能让 jwt 登录失效的

- 代码实现

  导入依赖

  ```xml
  <dependency>
      <groupId>org.glassfish</groupId>
      <artifactId>javax.xml.bind</artifactId>
      <version>10.0-b28</version>
  </dependency>
  <dependency>
      <groupId>com.auth0</groupId>
      <artifactId>java-jwt</artifactId>
      <version>3.2.0</version>
  </dependency>
  <dependency>
      <groupId>io.jsonwebtoken</groupId>
      <artifactId>jjwt</artifactId>
      <version>0.7.0</version>
  </dependency>
  ```

  实现

  ```java
  public class JwtUtils {
  
      // 服务端存的密码（重要），一旦丢失整个系统就不安全了
      private static final String secret = "this is secret";
  
      /**
       * 签发 JWT
       *
       * @param id
       * @param subject   可以是JSON数据，尽可能少
       * @param ttlMillis 存活时间
       */
      public static String createJWT(String id, String subject, long ttlMillis) {
          SignatureAlgorithm signatureAlgorithm = SignatureAlgorithm.HS256;
          long nowMillis = System.currentTimeMillis();
          Date now = new Date(nowMillis);
          JwtBuilder builder = Jwts.builder()
                  .setId(id)
                  .setSubject(subject)   // 主题
                  .setIssuer("user")     // 签发者
                  .setIssuedAt(now)      // 签发时间
                  .signWith(signatureAlgorithm, generalKey()); // 签名算法以及密匙
          if (ttlMillis >= 0) {
              long expMillis = nowMillis + ttlMillis;
              Date expDate = new Date(expMillis);
              builder.setExpiration(expDate); // 过期时间
          }
          return builder.compact();
      }
  
      /**
       * 验证 JWT
       */
      public static boolean validateJWT(String jwt) {
          Claims claims;
          try {
              claims = Jwts.parser()
                      .setSigningKey(generalKey())
                      .parseClaimsJws(jwt)
                      .getBody();
              System.out.println(claims);
              return true;
          } catch (Exception e) {
              return false;
          }
      }
  
      public static SecretKey generalKey() {
          byte[] encodedKey = Base64.decode(secret);
          SecretKey key = new SecretKeySpec(encodedKey, 0, encodedKey.length, "AES");
          return key;
      }
  
      public static void main(String[] args) throws Base64DecodingException {
          // 用户登录成功之后创建 jwt
          String jwt = createJWT("100", "{\"name\": \"tony\" }", 30000);
          System.out.println(jwt);
  
          // 用户之后访问后台的时候使用之前的 jwt 进行验证
          if (validateJWT(jwt)) {
              System.out.println("验证成功");
          } else {
              System.out.println("验证失败");
          }
      }
  }
  ```

  